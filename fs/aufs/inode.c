/*
 * Copyright (C) 2005-2016 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * inode functions
 */

#include "aufs.h"

struct inode *au_igrab(struct inode *inode)
{
	if (inode) {
		AuDebugOn(!atomic_read(&inode->i_count));
		ihold(inode);
	}
	return inode;
}

static void au_refresh_hinode_attr(struct inode *inode, int do_version)
{
	au_cpup_attr_all(inode, /*force*/0);
	au_update_iigen(inode, /*half*/1);
	if (do_version)
		inode->i_version++;
}

static int au_ii_refresh(struct inode *inode, int *update)
{
	int err, e;
	umode_t type;
	aufs_bindex_t bindex, new_bindex;
	struct super_block *sb;
	struct au_iinfo *iinfo;
	struct au_hinode *p, *q, tmp;

	AuDebugOn(is_bad_inode(inode));
	IiMustWriteLock(inode);

	*update = 0;
	sb = inode->i_sb;
	type = inode->i_mode & S_IFMT;
	iinfo = au_ii(inode);
	err = au_hinode_realloc(iinfo, au_sbbot(sb) + 1);
	if (unlikely(err))
		goto out;

	AuDebugOn(iinfo->ii_btop < 0);
	p = au_hinode(iinfo, iinfo->ii_btop);
	for (bindex = iinfo->ii_btop; bindex <= iinfo->ii_bbot;
	     bindex++, p++) {
		if (!p->hi_inode)
			continue;

		AuDebugOn(type != (p->hi_inode->i_mode & S_IFMT));
		new_bindex = au_br_index(sb, p->hi_id);
		if (new_bindex == bindex)
			continue;

		if (new_bindex < 0) {
			*update = 1;
			au_hiput(p);
			p->hi_inode = NULL;
			continue;
		}

		if (new_bindex < iinfo->ii_btop)
			iinfo->ii_btop = new_bindex;
		if (iinfo->ii_bbot < new_bindex)
			iinfo->ii_bbot = new_bindex;
		/* swap two lower inode, and loop again */
		q = au_hinode(iinfo, new_bindex);
		tmp = *q;
		*q = *p;
		*p = tmp;
		if (tmp.hi_inode) {
			bindex--;
			p--;
		}
	}
	au_update_ibrange(inode, /*do_put_zero*/0);
	e = au_dy_irefresh(inode);
	if (unlikely(e && !err))
		err = e;

out:
	AuTraceErr(err);
	return err;
}

void au_refresh_iop(struct inode *inode, int force_getattr)
{
	int type;
	struct au_sbinfo *sbi = au_sbi(inode->i_sb);
	const struct inode_operations *iop
		= force_getattr ? aufs_iop : sbi->si_iop_array;

	if (inode->i_op == iop)
		return;

	switch (inode->i_mode & S_IFMT) {
	case S_IFDIR:
		type = AuIop_DIR;
		break;
	case S_IFLNK:
		type = AuIop_SYMLINK;
		break;
	default:
		type = AuIop_OTHER;
		break;
	}

	inode->i_op = iop + type;
	/* unnecessary smp_wmb() */
}

int au_refresh_hinode_self(struct inode *inode)
{
	int err, update;

	err = au_ii_refresh(inode, &update);
	if (!err)
		au_refresh_hinode_attr(inode, update && S_ISDIR(inode->i_mode));

	AuTraceErr(err);
	return err;
}

int au_refresh_hinode(struct inode *inode, struct dentry *dentry)
{
	int err, e, update;
	unsigned int flags;
	umode_t mode;
	aufs_bindex_t bindex, bbot;
	unsigned char isdir;
	struct au_hinode *p;
	struct au_iinfo *iinfo;

	err = au_ii_refresh(inode, &update);
	if (unlikely(err))
		goto out;

	update = 0;
	iinfo = au_ii(inode);
	p = au_hinode(iinfo, iinfo->ii_btop);
	mode = (inode->i_mode & S_IFMT);
	isdir = S_ISDIR(mode);
	flags = au_hi_flags(inode, isdir);
	bbot = au_dbbot(dentry);
	for (bindex = au_dbtop(dentry); bindex <= bbot; bindex++) {
		struct inode *h_i, *h_inode;
		struct dentry *h_d;

		h_d = au_h_dptr(dentry, bindex);
		if (!h_d || d_is_negative(h_d))
			continue;

		h_inode = d_inode(h_d);
		AuDebugOn(mode != (h_inode->i_mode & S_IFMT));
		if (iinfo->ii_btop <= bindex && bindex <= iinfo->ii_bbot) {
			h_i = au_h_iptr(inode, bindex);
			if (h_i) {
				if (h_i == h_inode)
					continue;
				err = -EIO;
				break;
			}
		}
		if (bindex < iinfo->ii_btop)
			iinfo->ii_btop = bindex;
		if (iinfo->ii_bbot < bindex)
			iinfo->ii_bbot = bindex;
		au_set_h_iptr(inode, bindex, au_igrab(h_inode), flags);
		update = 1;
	}
	au_update_ibrange(inode, /*do_put_zero*/0);
	e = au_dy_irefresh(inode);
	if (unlikely(e && !err))
		err = e;
	if (!err)
		au_refresh_hinode_attr(inode, update && isdir);

out:
	AuTraceErr(err);
	return err;
}

static int set_inode(struct inode *inode, struct dentry *dentry)
{
	int err;
	unsigned int flags;
	umode_t mode;
	aufs_bindex_t bindex, btop, btail;
	unsigned char isdir;
	struct dentry *h_dentry;
	struct inode *h_inode;
	struct au_iinfo *iinfo;
	struct inode_operations *iop;

	IiMustWriteLock(inode);

	err = 0;
	isdir = 0;
	iop = au_sbi(inode->i_sb)->si_iop_array;
	btop = au_dbtop(dentry);
	h_dentry = au_h_dptr(dentry, btop);
	h_inode = d_inode(h_dentry);
	mode = h_inode->i_mode;
	switch (mode & S_IFMT) {
	case S_IFREG:
		btail = au_dbtail(dentry);
		inode->i_op = iop + AuIop_OTHER;
		inode->i_fop = &aufs_file_fop;
		err = au_dy_iaop(inode, btop, h_inode);
		if (unlikely(err))
			goto out;
		break;
	case S_IFDIR:
		isdir = 1;
		btail = au_dbtaildir(dentry);
		inode->i_op = iop + AuIop_DIR;
		inode->i_fop = &aufs_dir_fop;
		break;
	case S_IFLNK:
		btail = au_dbtail(dentry);
		inode->i_op = iop + AuIop_SYMLINK;
		break;
	case S_IFBLK:
	case S_IFCHR:
	case S_IFIFO:
	case S_IFSOCK:
		btail = au_dbtail(dentry);
		inode->i_op = iop + AuIop_OTHER;
		init_special_inode(inode, mode, h_inode->i_rdev);
		break;
	default:
		AuIOErr("Unknown file type 0%o\n", mode);
		err = -EIO;
		goto out;
	}

	/* do not set hnotify for whiteouted dirs (SHWH mode) */
	flags = au_hi_flags(inode, isdir);
	if (au_opt_test(au_mntflags(dentry->d_sb), SHWH)
	    && au_ftest_hi(flags, HNOTIFY)
	    && dentry->d_name.len > AUFS_WH_PFX_LEN
	    && !memcmp(dentry->d_name.name, AUFS_WH_PFX, AUFS_WH_PFX_LEN))
		au_fclr_hi(flags, HNOTIFY);
	iinfo = au_ii(inode);
	iinfo->ii_btop = btop;
	iinfo->ii_bbot = btail;
	for (bindex = btop; bindex <= btail; bindex++) {
		h_dentry = au_h_dptr(dentry, bindex);
		if (h_dentry)
			au_set_h_iptr(inode, bindex,
				      au_igrab(d_inode(h_dentry)), flags);
	}
	au_cpup_attr_all(inode, /*force*/1);
	/*
	 * to force calling aufs_get_acl() every time,
	 * do not call cache_no_acl() for aufs inode.
	 */

out:
	return err;
}

/*
 * successful returns with iinfo write_locked
 * minus: errno
 * zero: success, matched
 * plus: no error, but unmatched
 */
static int reval_inode(struct inode *inode, struct dentry *dentry)
{
	int err;
	unsigned int gen, igflags;
	aufs_bindex_t bindex, bbot;
	struct inode *h_inode, *h_dinode;
	struct dentry *h_dentry;

	/*
	 * before this function, if aufs got any iinfo lock, it must be only
	 * one, the parent dir.
	 * it can happen by UDBA and the obsoleted inode number.
	 */
	err = -EIO;
	if (unlikely(inode->i_ino == parent_ino(dentry)))
		goto out;

	err = 1;
	ii_write_lock_new_child(inode);
	h_dentry = au_h_dptr(dentry, au_dbtop(dentry));
	h_dinode = d_inode(h_dentry);
	bbot = au_ibbot(inode);
	for (bindex = au_ibtop(inode); bindex <= bbot; bindex++) {
		h_inode = au_h_iptr(inode, bindex);
		if (!h_inode || h_inode != h_dinode)
			continue;

		err = 0;
		gen = au_iigen(inode, &igflags);
		if (gen == au_digen(dentry)
		    && !au_ig_ftest(igflags, HALF_REFRESHED))
			break;

		/* fully refresh inode using dentry */
		err = au_refresh_hinode(inode, dentry);
		if (!err)
			au_update_iigen(inode, /*half*/0);
		break;
	}

	if (unlikely(err))
		ii_write_unlock(inode);
out:
	return err;
}

int au_ino(struct super_block *sb, aufs_bindex_t bindex, ino_t h_ino,
	   unsigned int d_type, ino_t *ino)
{
	int err;
	struct mutex *mtx;

	/* prevent hardlinked inode number from race condition */
	mtx = NULL;
	if (d_type != DT_DIR) {
		mtx = &au_sbr(sb, bindex)->br_xino.xi_nondir_mtx;
		mutex_lock(mtx);
	}
	err = au_xino_read(sb, bindex, h_ino, ino);
	if (unlikely(err))
		goto out;

	if (!*ino) {
		err = -EIO;
		*ino = au_xino_new_ino(sb);
		if (unlikely(!*ino))
			goto out;
		err = au_xino_write(sb, bindex, h_ino, *ino);
		if (unlikely(err))
			goto out;
	}

out:
	if (mtx)
		mutex_unlock(mtx);
	return err;
}

/* successful returns with iinfo write_locked */
/* todo: return with unlocked? */
struct inode *au_new_inode(struct dentry *dentry, int must_new)
{
	struct inode *inode, *h_inode;
	struct dentry *h_dentry;
	struct super_block *sb;
	struct mutex *mtx;
	ino_t h_ino, ino;
	int err;
	aufs_bindex_t btop;

	sb = dentry->d_sb;
	btop = au_dbtop(dentry);
	h_dentry = au_h_dptr(dentry, btop);
	h_inode = d_inode(h_dentry);
	h_ino = h_inode->i_ino;

	/*
	 * stop 'race'-ing between hardlinks under different
	 * parents.
	 */
	mtx = NULL;
	if (!d_is_dir(h_dentry))
		mtx = &au_sbr(sb, btop)->br_xino.xi_nondir_mtx;

new_ino:
	if (mtx)
		mutex_lock(mtx);
	err = au_xino_read(sb, btop, h_ino, &ino);
	inode = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	if (!ino) {
		ino = au_xino_new_ino(sb);
		if (unlikely(!ino)) {
			inode = ERR_PTR(-EIO);
			goto out;
		}
	}

	AuDbg("i%lu\n", (unsigned long)ino);
	inode = au_iget_locked(sb, ino);
	err = PTR_ERR(inode);
	if (IS_ERR(inode))
		goto out;

	AuDbg("%lx, new %d\n", inode->i_state, !!(inode->i_state & I_NEW));
	if (inode->i_state & I_NEW) {
		ii_write_lock_new_child(inode);
		err = set_inode(inode, dentry);
		if (!err) {
			unlock_new_inode(inode);
			goto out; /* success */
		}

		/*
		 * iget_failed() calls iput(), but we need to call
		 * ii_write_unlock() after iget_failed(). so dirty hack for
		 * i_count.
		 */
		atomic_inc(&inode->i_count);
		iget_failed(inode);
		ii_write_unlock(inode);
		au_xino_write(sb, btop, h_ino, /*ino*/0);
		/* ignore this error */
		goto out_iput;
	} else if (!must_new && !IS_DEADDIR(inode) && inode->i_nlink) {
		/*
		 * horrible race condition between lookup, readdir and copyup
		 * (or something).
		 */
		if (mtx)
			mutex_unlock(mtx);
		err = reval_inode(inode, dentry);
		if (unlikely(err < 0)) {
			mtx = NULL;
			goto out_iput;
		}

		if (!err) {
			mtx = NULL;
			goto out; /* success */
		} else if (mtx)
			mutex_lock(mtx);
	}

	if (unlikely(au_test_fs_unique_ino(h_inode)))
		AuWarn1("Warning: Un-notified UDBA or repeatedly renamed dir,"
			" b%d, %s, %pd, hi%lu, i%lu.\n",
			btop, au_sbtype(h_dentry->d_sb), dentry,
			(unsigned long)h_ino, (unsigned long)ino);
	ino = 0;
	err = au_xino_write(sb, btop, h_ino, /*ino*/0);
	if (!err) {
		iput(inode);
		if (mtx)
			mutex_unlock(mtx);
		goto new_ino;
	}

out_iput:
	iput(inode);
	inode = ERR_PTR(err);
out:
	if (mtx)
		mutex_unlock(mtx);
	return inode;
}

/* ---------------------------------------------------------------------- */

int au_test_ro(struct super_block *sb, aufs_bindex_t bindex,
	       struct inode *inode)
{
	int err;
	struct inode *hi;

	err = au_br_rdonly(au_sbr(sb, bindex));

	/* pseudo-link after flushed may happen out of bounds */
	if (!err
	    && inode
	    && au_ibtop(inode) <= bindex
	    && bindex <= au_ibbot(inode)) {
		/*
		 * permission check is unnecessary since vfsub routine
		 * will be called later
		 */
		hi = au_h_iptr(inode, bindex);
		if (hi)
			err = IS_IMMUTABLE(hi) ? -EROFS : 0;
	}

	return err;
}

int au_test_h_perm(struct inode *h_inode, int mask)
{
	if (uid_eq(current_fsuid(), GLOBAL_ROOT_UID))
		return 0;
	return inode_permission(h_inode, mask);
}

int au_test_h_perm_sio(struct inode *h_inode, int mask)
{
	if (au_test_nfs(h_inode->i_sb)
	    && (mask & MAY_WRITE)
	    && S_ISDIR(h_inode->i_mode))
		mask |= MAY_READ; /* force permission check */
	return au_test_h_perm(h_inode, mask);
}
