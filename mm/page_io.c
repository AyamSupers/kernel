// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/mm/page_io.c
 *
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 *
 *  Swap reorganised 29.12.95, 
 *  Asynchronous swapping added 30.12.95. Stephen Tweedie
 *  Removed race in async swapping. 14.4.1996. Bruno Haible
 *  Add swap of shared pages through the page cache. 20.2.1998. Stephen Tweedie
 *  Always use brw_page, life becomes simpler. 12 May 1998 Eric Biederman
 */

#include <linux/mm.h>
#include <linux/kernel_stat.h>
#include <linux/gfp.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/bio.h>
#include <linux/swapops.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/psi.h>
#include <linux/uio.h>
#include <linux/sched/task.h>
#include <linux/delayacct.h>
#include <linux/zswap.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include "swap.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/mm.h>

static void __end_swap_bio_write(struct bio *bio)
{
	struct folio *folio = bio_first_folio_all(bio);

	if (bio->bi_status) {
		/*
		 * We failed to write the page out to swap-space.
		 * Re-dirty the page in order to avoid it being reclaimed.
		 * Also print a dire warning that things will go BAD (tm)
		 * very quickly.
		 *
		 * Also clear PG_reclaim to avoid folio_rotate_reclaimable()
		 */
		folio_mark_dirty(folio);
		pr_alert_ratelimited("Write-error on swap-device (%u:%u:%llu)\n",
				     MAJOR(bio_dev(bio)), MINOR(bio_dev(bio)),
				     (unsigned long long)bio->bi_iter.bi_sector);
		folio_clear_reclaim(folio);
	}
	folio_end_writeback(folio);
}

static void end_swap_bio_write(struct bio *bio)
{
	__end_swap_bio_write(bio);
	bio_put(bio);
}

static void __end_swap_bio_read(struct bio *bio)
{
	struct folio *folio = bio_first_folio_all(bio);

	if (bio->bi_status) {
		pr_alert_ratelimited("Read-error on swap-device (%u:%u:%llu)\n",
				     MAJOR(bio_dev(bio)), MINOR(bio_dev(bio)),
				     (unsigned long long)bio->bi_iter.bi_sector);
	} else {
		folio_mark_uptodate(folio);
	}
	folio_unlock(folio);
}

static void end_swap_bio_read(struct bio *bio)
{
	__end_swap_bio_read(bio);
	bio_put(bio);
}

int generic_swapfile_activate(struct swap_info_struct *sis,
				struct file *swap_file,
				sector_t *span)
{
	struct address_space *mapping = swap_file->f_mapping;
	struct inode *inode = mapping->host;
	unsigned blocks_per_page;
	unsigned long page_no;
	unsigned blkbits;
	sector_t probe_block;
	sector_t last_block;
	sector_t lowest_block = -1;
	sector_t highest_block = 0;
	int nr_extents = 0;
	int ret;

	blkbits = inode->i_blkbits;
	blocks_per_page = PAGE_SIZE >> blkbits;

	/*
	 * Map all the blocks into the extent tree.  This code doesn't try
	 * to be very smart.
	 */
	probe_block = 0;
	page_no = 0;
	last_block = i_size_read(inode) >> blkbits;
	while ((probe_block + blocks_per_page) <= last_block &&
			page_no < sis->max) {
		unsigned block_in_page;
		sector_t first_block;

		cond_resched();

		first_block = probe_block;
		ret = bmap(inode, &first_block);
		if (ret || !first_block)
			goto bad_bmap;

		/*
		 * It must be PAGE_SIZE aligned on-disk
		 */
		if (first_block & (blocks_per_page - 1)) {
			probe_block++;
			goto reprobe;
		}

		for (block_in_page = 1; block_in_page < blocks_per_page;
					block_in_page++) {
			sector_t block;

			block = probe_block + block_in_page;
			ret = bmap(inode, &block);
			if (ret || !block)
				goto bad_bmap;

			if (block != first_block + block_in_page) {
				/* Discontiguity */
				probe_block++;
				goto reprobe;
			}
		}

		first_block >>= (PAGE_SHIFT - blkbits);
		if (page_no) {	/* exclude the header page */
			if (first_block < lowest_block)
				lowest_block = first_block;
			if (first_block > highest_block)
				highest_block = first_block;
		}

		/*
		 * We found a PAGE_SIZE-length, PAGE_SIZE-aligned run of blocks
		 */
		ret = add_swap_extent(sis, page_no, 1, first_block);
		if (ret < 0)
			goto out;
		nr_extents += ret;
		page_no++;
		probe_block += blocks_per_page;
reprobe:
		continue;
	}
	ret = nr_extents;
	*span = 1 + highest_block - lowest_block;
	if (page_no == 0)
		page_no = 1;	/* force Empty message */
	sis->max = page_no;
	sis->pages = page_no - 1;
	sis->highest_bit = page_no - 1;
out:
	return ret;
bad_bmap:
	pr_err("swapon: swapfile has holes\n");
	ret = -EINVAL;
	goto out;
}

/*
 * kcompressd: offload compression/swapout from kswapd (mm/page_io.c)
 *
 * Backport of firelzrd/kcompressd-unofficial 0.5 (itself built on
 * Kcompressd by Qun-Wei Lin, MediaTek) to 6.6.
 *
 * Per-node state is kept in a private static array here instead of
 * inside pg_data_t: pg_data_t is part of this kernel's frozen GKI
 * ABI surface and has only one ANDROID_KABI_RESERVE slot left, far
 * too small for a waitqueue + kfifo + lock + task pointer.  Keeping
 * the new state entirely outside pg_data_t means its layout, and
 * therefore vendor .ko compatibility, is untouched.
 */
#define KCOMPRESS_FIFO_SIZE 256

struct kcompressd_data {
	wait_queue_head_t wait;
	struct task_struct *task;
	struct kfifo fifo;
	spinlock_t fifo_lock;
};

static struct kcompressd_data kcompressd_nodes[MAX_NUMNODES];

int vm_kcompressd = 24;

/*
 * do_swapout() - write a folio to swap, mirroring the synchronous
 * swap_writepage() path below (including its zswap writeback dance).
 * The folio must be locked on entry; it is unlocked and its extra
 * reference dropped before returning.
 */
static inline void do_swapout(struct folio *folio)
{
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_NONE,
		.nr_to_write = SWAP_CLUSTER_MAX,
		.range_start = 0,
		.range_end = LLONG_MAX,
		.for_reclaim = 1,
	};

	if (zswap_store(folio)) {
		folio_start_writeback(folio);
		folio_unlock(folio);
		folio_end_writeback(folio);
	} else {
		/*
		 * Unlocks the folio internally and fires the android
		 * vendor hooks (trace_android_vh_swap_writepage_*).
		 */
		__swap_writepage(&folio->page, &wbc);
	}

	folio_put(folio);
}

/*
 * kcompressd_store() - try to hand a folio off to kcompressd instead
 * of compressing/writing it synchronously on the kswapd stack.
 * Returns true if the folio was queued (or synchronously flushed as
 * part of making room in the queue); false means the caller must
 * fall back to the normal synchronous path.
 */
static bool kcompressd_store(struct folio *folio)
{
	int nid = folio_nid(folio);
	struct kcompressd_data *kd = &kcompressd_nodes[nid];
	unsigned int ret, sysctl_kcompressd = READ_ONCE(vm_kcompressd);
	struct folio *head = NULL;
	unsigned long flags;

	/* Only kswapd may offload; direct reclaim stays synchronous */
	if (!current_is_kswapd())
		return false;

	if (!sysctl_kcompressd || unlikely(!kd->task))
		return false;

	/* Only anon folios go through this path */
	if (!folio_test_anon(folio))
		return false;

	/*
	 * Only worth offloading for synchronous-IO swap backends (zram).
	 * zswap eligibility itself is decided inside zswap_store(),
	 * called from do_swapout() on the kcompressd thread; 6.6 has no
	 * zswap_is_enabled() / mem_cgroup_zswap_writeback_enabled()
	 * helpers, so there is no separate zswap pre-check here.
	 */
	if (!data_race(swp_swap_info(folio->swap)->flags & SWP_SYNCHRONOUS_IO))
		return false;

	/*
	 * If the FIFO is nearly full, synchronously flush the oldest
	 * entry first so ordering/freshness is preserved instead of
	 * silently dropping to synchronous mode for the new folio.
	 */
	spin_lock_irqsave(&kd->fifo_lock, flags);
	if (kfifo_len(&kd->fifo) >= sysctl_kcompressd * sizeof(folio) &&
			unlikely(!kfifo_out(&kd->fifo, &head, sizeof(folio)))) {
		spin_unlock_irqrestore(&kd->fifo_lock, flags);
		return false;
	}
	spin_unlock_irqrestore(&kd->fifo_lock, flags);

	folio_get(folio);
	ret = kfifo_in(&kd->fifo, &folio, sizeof(folio));
	if (likely(ret))
		wake_up_interruptible(&kd->wait);
	else
		folio_put(folio);

	if (head)
		do_swapout(head);

	return ret;
}

static int kcompressd(void *p)
{
	int nid = (long)p;
	struct kcompressd_data *kd = &kcompressd_nodes[nid];
	struct folio *folio;

	/*
	 * PF_MEMALLOC: allow allocating memory (compression buffers)
	 * without being throttled by the same reclaim we're helping.
	 * PF_KSWAPD: identify as kswapd-class to the rest of mm/.
	 */
	current->flags |= PF_MEMALLOC | PF_KSWAPD;

	while (!kthread_should_stop()) {
		wait_event_interruptible(kd->wait, !kfifo_is_empty(&kd->fifo));

		while (kfifo_out_locked(&kd->fifo, &folio, sizeof(folio), &kd->fifo_lock))
			do_swapout(folio);
	}
	return 0;
}

void kcompressd_run(int nid)
{
	struct kcompressd_data *kd = &kcompressd_nodes[nid];
	int ret;

	if (kd->task)
		return;

	ret = kfifo_alloc(&kd->fifo, KCOMPRESS_FIFO_SIZE * sizeof(struct folio *),
			   GFP_KERNEL);
	if (ret) {
		pr_err("kcompressd: fail to kfifo_alloc on node %d\n", nid);
		return;
	}
	init_waitqueue_head(&kd->wait);
	spin_lock_init(&kd->fifo_lock);

	kd->task = kthread_run(kcompressd, (void *)(long)nid, "kcompressd%d", nid);
	if (IS_ERR(kd->task)) {
		pr_err("kcompressd: failed to start on node %d, ret=%ld\n",
			nid, PTR_ERR(kd->task));
		kd->task = NULL;
		kfifo_free(&kd->fifo);
	} else {
		pr_info("kcompressd: started on node %d\n", nid);
	}
}

void kcompressd_stop(int nid)
{
	struct kcompressd_data *kd = &kcompressd_nodes[nid];

	if (kd->task) {
		kthread_stop(kd->task);
		kd->task = NULL;
		kfifo_free(&kd->fifo);
	}
}

/*
 * We may have stale swap cache pages in memory: notice
 * them here and get rid of the unnecessary final write.
 */
int swap_writepage(struct page *page, struct writeback_control *wbc)
{
	struct folio *folio = page_folio(page);
	int ret;

	if (folio_free_swap(folio)) {
		folio_unlock(folio);
		return 0;
	}
	/*
	 * Arch code may have to preserve more data than just the page
	 * contents, e.g. memory tags.
	 */
	ret = arch_prepare_to_swap(folio);
	if (ret) {
		folio_mark_dirty(folio);
		folio_unlock(folio);
		return ret;
	}

	/*
	 * Compression (zswap/zram) may block on rmap/unmap; try to hand
	 * the folio off to kcompressd when possible.
	 */
	if (kcompressd_store(folio))
		return 0;

	if (zswap_store(folio)) {
		folio_start_writeback(folio);
		folio_unlock(folio);
		folio_end_writeback(folio);
		return 0;
	}
	__swap_writepage(&folio->page, wbc);
	return 0;
}

static inline void count_swpout_vm_event(struct folio *folio)
{
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	if (unlikely(folio_test_pmd_mappable(folio))) {
		count_memcg_folio_events(folio, THP_SWPOUT, 1);
		count_vm_event(THP_SWPOUT);
	}
	count_mthp_stat(folio_order(folio), MTHP_STAT_SWPOUT);
#endif
	count_vm_events(PSWPOUT, folio_nr_pages(folio));
}

#if defined(CONFIG_MEMCG) && defined(CONFIG_BLK_CGROUP)
static void bio_associate_blkg_from_page(struct bio *bio, struct folio *folio)
{
	struct cgroup_subsys_state *css;
	struct mem_cgroup *memcg;

	memcg = folio_memcg(folio);
	if (!memcg)
		return;

	rcu_read_lock();
	css = cgroup_e_css(memcg->css.cgroup, &io_cgrp_subsys);
	bio_associate_blkg_from_css(bio, css);
	rcu_read_unlock();
}
#else
#define bio_associate_blkg_from_page(bio, folio)		do { } while (0)
#endif /* CONFIG_MEMCG && CONFIG_BLK_CGROUP */

struct swap_iocb {
	struct kiocb		iocb;
	struct bio_vec		bvec[SWAP_CLUSTER_MAX];
	int			pages;
	int			len;
};
static mempool_t *sio_pool;

int sio_pool_init(void)
{
	if (!sio_pool) {
		mempool_t *pool = mempool_create_kmalloc_pool(
			SWAP_CLUSTER_MAX, sizeof(struct swap_iocb));
		if (cmpxchg(&sio_pool, NULL, pool))
			mempool_destroy(pool);
	}
	if (!sio_pool)
		return -ENOMEM;
	return 0;
}

static void sio_write_complete(struct kiocb *iocb, long ret)
{
	struct swap_iocb *sio = container_of(iocb, struct swap_iocb, iocb);
	struct page *page = sio->bvec[0].bv_page;
	int p;

	if (ret != sio->len) {
		/*
		 * In the case of swap-over-nfs, this can be a
		 * temporary failure if the system has limited
		 * memory for allocating transmit buffers.
		 * Mark the page dirty and avoid
		 * folio_rotate_reclaimable but rate-limit the
		 * messages but do not flag PageError like
		 * the normal direct-to-bio case as it could
		 * be temporary.
		 */
		pr_err_ratelimited("Write error %ld on dio swapfile (%llu)\n",
				   ret, page_file_offset(page));
		for (p = 0; p < sio->pages; p++) {
			page = sio->bvec[p].bv_page;
			set_page_dirty(page);
			ClearPageReclaim(page);
		}
	}

	for (p = 0; p < sio->pages; p++)
		end_page_writeback(sio->bvec[p].bv_page);

	mempool_free(sio, sio_pool);
}

static void swap_writepage_fs(struct page *page, struct writeback_control *wbc)
{
	struct swap_iocb *sio = NULL;
	struct swap_info_struct *sis = page_swap_info(page);
	struct file *swap_file = sis->swap_file;
	loff_t pos = page_file_offset(page);

	count_swpout_vm_event(page_folio(page));
	set_page_writeback(page);
	unlock_page(page);
	if (wbc->swap_plug)
		sio = *wbc->swap_plug;
	if (sio) {
		if (sio->iocb.ki_filp != swap_file ||
		    sio->iocb.ki_pos + sio->len != pos) {
			swap_write_unplug(sio);
			sio = NULL;
		}
	}
	if (!sio) {
		sio = mempool_alloc(sio_pool, GFP_NOIO);
		init_sync_kiocb(&sio->iocb, swap_file);
		sio->iocb.ki_complete = sio_write_complete;
		sio->iocb.ki_pos = pos;
		sio->pages = 0;
		sio->len = 0;
	}
	bvec_set_page(&sio->bvec[sio->pages], page, thp_size(page), 0);
	sio->len += thp_size(page);
	sio->pages += 1;
	if (sio->pages == ARRAY_SIZE(sio->bvec) || !wbc->swap_plug) {
		swap_write_unplug(sio);
		sio = NULL;
	}
	if (wbc->swap_plug)
		*wbc->swap_plug = sio;
}

static void swap_writepage_bdev_sync(struct page *page,
		struct writeback_control *wbc, struct swap_info_struct *sis)
{
	struct bio_vec bv;
	struct bio bio;
	struct folio *folio = page_folio(page);

	bio_init(&bio, sis->bdev, &bv, 1,
		 REQ_OP_WRITE | REQ_SWAP | wbc_to_write_flags(wbc));
	bio.bi_iter.bi_sector = swap_page_sector(page);
	__bio_add_page(&bio, page, thp_size(page), 0);

	bio_associate_blkg_from_page(&bio, folio);
	count_swpout_vm_event(folio);

	folio_start_writeback(folio);
	folio_unlock(folio);

	submit_bio_wait(&bio);
	__end_swap_bio_write(&bio);
}

static void swap_writepage_bdev_async(struct page *page,
		struct writeback_control *wbc, struct swap_info_struct *sis)
{
	struct bio *bio;
	struct folio *folio = page_folio(page);

	bio = bio_alloc(sis->bdev, 1,
			REQ_OP_WRITE | REQ_SWAP | wbc_to_write_flags(wbc),
			GFP_NOIO);
	bio->bi_iter.bi_sector = swap_page_sector(page);
	bio->bi_end_io = end_swap_bio_write;
	__bio_add_page(bio, page, thp_size(page), 0);

	bio_associate_blkg_from_page(bio, folio);
	count_swpout_vm_event(folio);
	folio_start_writeback(folio);
	folio_unlock(folio);
	submit_bio(bio);
}

void __swap_writepage(struct page *page, struct writeback_control *wbc)
{
	struct swap_info_struct *sis = page_swap_info(page);
	unsigned long sis_flags = 0;
	unsigned long swap_writepage_start;

	VM_BUG_ON_PAGE(!PageSwapCache(page), page);
	trace_android_vh_swap_writepage_start(&swap_writepage_start);
	/*
	 * ->flags can be updated non-atomicially (scan_swap_map_slots),
	 * but that will never affect __SWP_WRITE_SYNCHRONOUS_IO, so the data_race
	 * is safe.
	 */
	sis_flags = data_race(sis->flags);
	trace_android_vh_swap_writepage(&sis_flags, page);
	if (sis_flags & SWP_FS_OPS)
		swap_writepage_fs(page, wbc);
	else if (sis_flags & __SWP_WRITE_SYNCHRONOUS_IO)
		swap_writepage_bdev_sync(page, wbc, sis);
	else
		swap_writepage_bdev_async(page, wbc, sis);

	trace_android_vh_swap_writepage_end(page, wbc,
		swap_writepage_start);
}

void swap_write_unplug(struct swap_iocb *sio)
{
	struct iov_iter from;
	struct address_space *mapping = sio->iocb.ki_filp->f_mapping;
	int ret;

	iov_iter_bvec(&from, ITER_SOURCE, sio->bvec, sio->pages, sio->len);
	ret = mapping->a_ops->swap_rw(&sio->iocb, &from);
	if (ret != -EIOCBQUEUED)
		sio_write_complete(&sio->iocb, ret);
}

static void sio_read_complete(struct kiocb *iocb, long ret)
{
	struct swap_iocb *sio = container_of(iocb, struct swap_iocb, iocb);
	int p;

	if (ret == sio->len) {
		for (p = 0; p < sio->pages; p++) {
			struct folio *folio = page_folio(sio->bvec[p].bv_page);

			folio_mark_uptodate(folio);
			folio_unlock(folio);
		}
		count_vm_events(PSWPIN, sio->pages);
	} else {
		for (p = 0; p < sio->pages; p++) {
			struct folio *folio = page_folio(sio->bvec[p].bv_page);

			folio_unlock(folio);
		}
		pr_alert_ratelimited("Read-error on swap-device\n");
	}
	mempool_free(sio, sio_pool);
}

static void swap_readpage_fs(struct page *page,
			     struct swap_iocb **plug)
{
	struct swap_info_struct *sis = page_swap_info(page);
	struct swap_iocb *sio = NULL;
	loff_t pos = page_file_offset(page);

	if (plug)
		sio = *plug;
	if (sio) {
		if (sio->iocb.ki_filp != sis->swap_file ||
		    sio->iocb.ki_pos + sio->len != pos) {
			swap_read_unplug(sio);
			sio = NULL;
		}
	}
	if (!sio) {
		sio = mempool_alloc(sio_pool, GFP_KERNEL);
		init_sync_kiocb(&sio->iocb, sis->swap_file);
		sio->iocb.ki_pos = pos;
		sio->iocb.ki_complete = sio_read_complete;
		sio->pages = 0;
		sio->len = 0;
	}
	bvec_set_page(&sio->bvec[sio->pages], page, thp_size(page), 0);
	sio->len += thp_size(page);
	sio->pages += 1;
	if (sio->pages == ARRAY_SIZE(sio->bvec) || !plug) {
		swap_read_unplug(sio);
		sio = NULL;
	}
	if (plug)
		*plug = sio;
}

static void swap_readpage_bdev_sync(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio_vec bv;
	struct bio bio;
	bool read = false;

	trace_android_rvh_swap_readpage_bdev_sync(sis->bdev,
		swap_page_sector(&folio->page) + get_start_sect(sis->bdev),
		&folio->page, &read);
	if (read) {
		count_vm_events(PSWPIN, folio_nr_pages(folio));
		return;
	}

	/*
	 * trace_android_vh_swap_readpage_bdev_sync is deprecated, and
	 * should not be carried over into later kernels.
	 * Use trace_android_rvh_swap_readpage_bdev_sync instead.
	 */
	trace_android_vh_swap_readpage_bdev_sync(sis->bdev,
		swap_page_sector(&folio->page) + get_start_sect(sis->bdev),
		&folio->page, &read);
	if (read) {
		count_vm_events(PSWPIN, folio_nr_pages(folio));
		return;
	}

	bio_init(&bio, sis->bdev, &bv, 1, REQ_OP_READ);
	bio.bi_iter.bi_sector = swap_page_sector(&folio->page);
	bio_add_folio_nofail(&bio, folio, folio_size(folio), 0);
	/*
	 * Keep this task valid during swap readpage because the oom killer may
	 * attempt to access it in the page fault retry time check.
	 */
	get_task_struct(current);
	count_vm_events(PSWPIN, folio_nr_pages(folio));
	submit_bio_wait(&bio);
	trace_android_rvh_swap_bio_charge(&bio);
	__end_swap_bio_read(&bio);
	put_task_struct(current);
}

static void swap_readpage_bdev_async(struct folio *folio,
		struct swap_info_struct *sis)
{
	struct bio *bio;

	bio = bio_alloc(sis->bdev, 1, REQ_OP_READ, GFP_KERNEL);
	bio->bi_iter.bi_sector = swap_page_sector(&folio->page);
	bio->bi_end_io = end_swap_bio_read;
	bio_add_folio_nofail(bio, folio, folio_size(folio), 0);
	count_vm_events(PSWPIN, folio_nr_pages(folio));
	submit_bio(bio);
}

void swap_readpage(struct page *page, bool synchronous, struct swap_iocb **plug)
{
	struct folio *folio = page_folio(page);
	struct swap_info_struct *sis = page_swap_info(page);
	bool workingset = folio_test_workingset(folio);
	unsigned long pflags;
	bool in_thrashing;
	unsigned long swapin_start;

	VM_BUG_ON_FOLIO(!folio_test_swapcache(folio) && !synchronous, folio);
	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	VM_BUG_ON_FOLIO(folio_test_uptodate(folio), folio);
	trace_android_vh_swapin_start(&swapin_start);

	/*
	 * Count submission time as memory stall and delay. When the device
	 * is congested, or the submitting cgroup IO-throttled, submission
	 * can be a significant part of overall IO time.
	 */
	if (workingset) {
		delayacct_thrashing_start(&in_thrashing);
		psi_memstall_enter(&pflags);
	}
	delayacct_swapin_start();

	if (zswap_load(folio)) {
		folio_mark_uptodate(folio);
		folio_unlock(folio);
	} else if (data_race(sis->flags & SWP_FS_OPS)) {
		swap_readpage_fs(page, plug);
	} else if (synchronous || (sis->flags & SWP_SYNCHRONOUS_IO)) {
		swap_readpage_bdev_sync(folio, sis);
	} else {
		swap_readpage_bdev_async(folio, sis);
	}

	if (workingset) {
		delayacct_thrashing_end(&in_thrashing);
		psi_memstall_leave(&pflags);
	}
	delayacct_swapin_end();
	trace_android_vh_swapin_end(folio, swapin_start);
}

void __swap_read_unplug(struct swap_iocb *sio)
{
	struct iov_iter from;
	struct address_space *mapping = sio->iocb.ki_filp->f_mapping;
	int ret;

	iov_iter_bvec(&from, ITER_DEST, sio->bvec, sio->pages, sio->len);
	ret = mapping->a_ops->swap_rw(&sio->iocb, &from);
	if (ret != -EIOCBQUEUED)
		sio_read_complete(&sio->iocb, ret);
}
