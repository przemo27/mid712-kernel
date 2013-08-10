/*
 * fs/sysfs/ram.c - sysfs ram file implementation
 *
 * Copyright (c) 2009 hsl 
 *
 * This file is released under the GPLv2.
 *
 * Please see Documentation/filesystems/sysfs.txt for more information.
 */

#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>

#define RAM_ENABLE                              1
#if RAM_ENABLE 
#define DEBUG
//#undef DEBUG
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/notifier.h>	/* For notifier support */
#include <linux/reboot.h>	/* For reboot_notifier stuff */
#include <linux/init.h>		/* For __init/__exit/... */
#include <asm/uaccess.h>

#include "sysfs.h"


#define RAM_MAX_BUFFER_SIZE                     (2*1024*1024)   // 2M
#define RAM_MAX_ONE_BUFFER_SIZE            (512*1024)   // 512 k

#define RMA_MAX_FILE_LEN                    128
#define RAM_MIN_BUFFER_ORDER            0 // 4 page , 16K,db *2 .
struct ram_buffer {
             unsigned int             flags ; /* updated , flushed , */
             ssize_t                    size; /* current size , inode size as the max size */
             struct list_head        list;  /* open list */
             char		   *buffer;
             int                          write_count;

             char                       dest_filename[RMA_MAX_FILE_LEN]; /* dest file name */
             char                       sys_filename[64]; /* sys file name */
             char                       proc_name[16]; /* proc name */
             int                          page_order; /* page order aligned */
             mode_t                  mode;
             struct file *            filp;     /* host file point */
};

static LIST_HEAD( ram_opend_list );
static DECLARE_MUTEX(mutex_ram_open);

#define RAM_MAX_SIZE( bb )             ( (1<<bb->page_order)*PAGE_SIZE )   
#define RAM_FL_LOADED                   (1<<0) 
#define RAM_FL_FLUSHED                  (1<<1)
#define RAM_FL_FILEEXIT                  (1<<2)
#define RAM_FL_CONFLUSH                (1<<3)
#define RAM_FL_CREATED                  (1<<4)
#define RAM_FL_NOTSUPPORT           (1<<5)


#define RAM_SYSFS_DIR                      "/sys/kernel/"

char default_config_filepath[RMA_MAX_FILE_LEN]= "/data/app/ram_config";
static int ram_configed = 1;    /* as a switch */
static unsigned long total_ram_size = 0;    /* as a switch */


const struct file_operations ram_rw_fops;
extern char *saved_command_line;

/**
 *  type: 0:sys file name , 1: dest filename 
 *
 */
static struct ram_buffer *ram_search_baking_file( const char* filename , int type )
{
        struct ram_buffer *bb;
        int     found = 0;
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                if( (type == 0 &&  !strcmp(bb->sys_filename , filename ) )
                    || (type == 1 &&  !strcmp(bb->dest_filename , filename ) ) ) {
                        found = 1;
                 //       printk("%s::found %s,bb=0x%p\n" ,__func__ , filename , bb );
                        break;
                }
        }
        up( &mutex_ram_open );
        if( found )
                return bb;
        return NULL;
}

static int ram_open(struct inode * inode, struct file * file)
{
	struct sysfs_dirent *attr_sd = file->f_path.dentry->d_fsdata;
	struct ram_attribute *attr = attr_sd->s_ram_attr.ram_attr;
	struct ram_buffer *bb = NULL;
             
                pr_debug("%s::attr=%s\n" , __func__ , attr->attr.name );
                /* ram file operations requires both @sd and its parent */
                if (!sysfs_get_active_two(attr_sd))
                        return -ENODEV;

                bb = ram_search_baking_file( attr->attr.name , 0 );
                if( !bb ) 
                        goto err_out;
                
                file->private_data = bb;
                file->f_pos = 0;
                file->f_version = 0;
	/* open succeeded, put active references */
	sysfs_put_active_two(attr_sd);
	return 0;
 err_out:
	sysfs_put_active_two(attr_sd);
	return -EIO;
}

static int ram_release(struct inode * inode, struct file * file)
{
                pr_debug("%s::file=%s\n" , __func__ , file->f_path.dentry->d_name.name );
	return 0;
}

static char* ram_alloc_buffer( struct ram_buffer *bb )
{
        char* buf = (char*)__get_free_pages(GFP_KERNEL , bb->page_order );
        if( !buf ) {
                printk("%s::alloc %d page failed\n" , __func__ , 1<<bb->page_order);
                return NULL;
        }
        memset( buf , 0 , 1<<bb->page_order );
        total_ram_size += RAM_MAX_SIZE( bb );
        return buf;
}

static void ram_free_buffer( struct ram_buffer *bb )
{
        char  *buf = bb->buffer;
        bb->buffer = NULL;
        free_pages( (unsigned long)buf , bb->page_order );
        bb->size = 0;
        total_ram_size -= RAM_MAX_SIZE( bb );
}

static inline size_t ram_get_total_size( void )
{
        return total_ram_size;
}

static int ram_loader_file( struct ram_buffer *bb )
{
        ssize_t nread = 0;
        loff_t			file_offset=0;
        mm_segment_t old_fs;
        struct file * filp;

        pr_debug("%s..%d:: bb flags=0x%x,order=%d,mode=0%o\n" ,__func__ ,__LINE__ , bb->flags , bb->page_order , bb->mode );
        if( (bb->flags & RAM_FL_LOADED) &&  bb->buffer )
                goto  loaded;
        if( !bb->buffer ) {
                bb->buffer = ram_alloc_buffer( bb );
                if( !bb->buffer ) {
                        goto out;
                }
        }
        
        if( bb->mode & O_TRUNC ) 
                goto filereaded;
        filp = filp_open( bb->dest_filename , O_RDONLY|O_SYNC,  0);
        if ( !IS_ERR(filp)) {
                nread =  RAM_MAX_SIZE( bb );
                old_fs = get_fs();
                set_fs(KERNEL_DS);
                nread = vfs_read(filp,bb->buffer ,nread, &file_offset);
                set_fs(old_fs);
                pr_debug("%s::read dest file: %s, nread=%d\n",__func__ , bb->dest_filename, nread);
                if( nread > 0 )
                {
                        bb->flags |= RAM_FL_FILEEXIT;
                }
                fput(filp);
        } else {
                printk("%s::dest file: %s not exist\n",__func__ , bb->dest_filename );
        }
filereaded:        
        bb->size = nread;
        bb->flags |= RAM_FL_LOADED;
loaded:        
        if( bb->filp ) {
                bb->filp->private_data = bb;
                bb->filp->f_ramop = &ram_rw_fops;
                pr_debug("%s..%d:: load file %s OK\n" ,__func__ ,__LINE__ , 
                        bb->dest_filename );
        }
        return 0;
out:
        return -EIO;
}

static int ram_save_file( struct ram_buffer *bb  )
{
        struct file * filp ;
        long rc;
        ssize_t nwrite;
        loff_t			file_offset = 0;
        mm_segment_t old_fs;

        if( bb->mode & O_TRUNC )
                return 0;
       
        filp = filp_open( bb->dest_filename , O_WRONLY|O_CREAT|O_TRUNC|O_SYNC ,  0);
        if (IS_ERR(filp)) {
	printk("%s::unable to create backing file: %s\n",__func__ , bb->dest_filename);
	return -ENOENT;
        } 
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        vfs_llseek(filp, 0, SEEK_SET );
        nwrite = vfs_write(filp,bb->buffer ,bb->size , &file_offset);
        set_fs(old_fs);
        if( nwrite != bb->size )
                printk("%s::write backing file: %s error , amont=%d , nwrite=%d\n",__func__ , bb->dest_filename,bb->size,nwrite);
        else {
                pr_debug("%s::write backing file: %s OK , nwrite=%d\n",__func__ , bb->dest_filename, nwrite);
                bb->flags |= RAM_FL_FLUSHED;
                rc = do_fsync(filp, 1);
                if (rc < 0)
                	printk(KERN_ERR "ums: Error syncing data (%ld)\n", rc);
        }
        fput(filp);
        return 0;
}

static int ram_save_config( const char* config_file , char * buf , int len )
{
        ssize_t write;
        loff_t			file_offset=0;
        mm_segment_t old_fs;
        struct file * filp;
        
        filp = filp_open( config_file , O_WRONLY|O_CREAT|O_TRUNC|O_SYNC ,  0777 );
        if (IS_ERR(filp)) {
                printk("%s::unable to open config file: %s\n",__func__ , config_file);
                return -ENOENT;
        }
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        write = vfs_write(filp, buf , len , &file_offset); /* 1 for '\0' */
        set_fs(old_fs);
        if( write != len ){
                printk("%s::write config file: %s failed, write=%d,want=%d\n",__func__ , config_file , write, len);
                goto out;
        } else {
                pr_debug("%s::write config file: %s OK , nwrite=%d\n",__func__ , config_file , write);
        }
        fput(filp);
        return 0;
out:
        fput(filp);
        return -EIO;    
}


/*
 * format : "xxxxk mod dest_name sys_name\n"
 *
 */
int ram_sync_config( void )
{
        struct ram_buffer *bb;
        char   *p;
        char *buf ;
        int     needconfig = 0;
        
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                if( !(bb->flags & RAM_FL_CONFLUSH) && !( bb->mode & O_TRUNC ) ) {
                       needconfig = 1;
                       break;
                }
        }
        up( &mutex_ram_open );
        
        if( !needconfig )
                return 0;
        buf = kzalloc( PAGE_SIZE*2 , GFP_KERNEL);
        if( !buf )
                return -ENOMEM;
        p = buf;
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                bb->flags |= RAM_FL_CONFLUSH;
                if( !( bb->mode & O_TRUNC )  ) 
                        p += sprintf( p , "%ldk 0%o %s %s\n" , RAM_MAX_SIZE(bb)>>10 ,
                                bb->mode , bb->dest_filename,bb->sys_filename );
        }
        up( &mutex_ram_open );
        if( p > buf ) {
                ram_save_config(default_config_filepath, buf , p-buf );
        }
        kfree( buf );
        return 0;
}
static void ram_list_sync( unsigned int count )
{
        struct ram_buffer *bb;
        pr_debug("%s:: sync count=%d\n" , __func__ , count );
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                if( !(bb->flags & RAM_FL_FLUSHED ) ) {
                        if( count && !ram_save_file( bb ) ) {
                                count--;
                        }
                }
        }
        up( &mutex_ram_open );
}

int ram_sync( void )
{
        ram_sync_config();
        ram_list_sync( 1 );
        return 0;
}

int ram_file_print( int sync  )
{
        struct ram_buffer *bb;
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                printk("name=%s:max size=%ld,nr write=%d,proc=%s,mod=0%o,size=%d,flag=0x%x\n" , 
                        bb->dest_filename, RAM_MAX_SIZE(bb) , bb->write_count , bb->proc_name , bb->mode ,
                        bb->size , bb->flags );
        }
        up( &mutex_ram_open );
        printk("total require memory:%dK \n" , ram_get_total_size()>>10 );
        if( sync ) {
                ram_sync_config();
                ram_list_sync( -1 );
        }
        return 0x101;
}

/**
 *	sysfs_create_ram_file - create ram file for object.
 *	@kobj:	object.
 *	@attr:	attribute descriptor.
 */

int sysfs_create_ram_file(struct kobject * kobj, struct ram_attribute * attr)
{
	BUG_ON(!kobj || !kobj->sd || !attr);
	return sysfs_add_file(kobj->sd, &attr->attr, SYSFS_KOBJ_RAM_ATTR);
}


/**
 *	sysfs_remove_ram_file - remove ram file for object.
 *	@kobj:	object.
 *	@attr:	attribute descriptor.
 */

void sysfs_remove_ram_file(struct kobject * kobj, struct ram_attribute * attr)
{
	sysfs_hash_and_remove(kobj->sd, attr->attr.name);
}

/**
 *      create ram file filename for destfile.
 *      and write config info to .config file for next start.
 *
 *
 */ 

static void ram_get_filename( struct ram_buffer *bb )
{
        char * lastname = strrchr( bb->dest_filename , '/' );
        
        if( lastname ) 
                lastname++;
        else 
                lastname = bb->dest_filename ;
        strcpy( bb->sys_filename , lastname );
        strcat( bb->sys_filename , "s" );
}

static inline int ram_get_pageorder(ssize_t size )
{
        int order = RAM_MIN_BUFFER_ORDER;
        while( PAGE_SIZE * (1<<order) < size )
                order++;
        return order;
}

#if 0 /* 20091130,HSL@RK , no use */
int ram_create_file(  struct ram_buffer *bb  )
{
        int     rc = 0;
        struct ram_attribute    *attr;

        if( bb->flags & RAM_FL_CREATED )
                goto created;
        pr_debug("%s:: file=%s\n" ,__func__ , bb->sys_filename );
        attr = kmalloc( sizeof *attr , GFP_KERNEL );
        if( !attr ) 
                return -ENOMEM;
        attr->attr.owner = THIS_MODULE;
        attr->attr.mode = bb->mode;
        attr->attr.name = bb->sys_filename;
        attr->size = RAM_MAX_SIZE( bb );
        rc = sysfs_create_ram_file(kernel_kobj, attr) ;
        if( rc ) {
                goto err_out0;
       }
       bb->flags |= RAM_FL_CREATED;
created:       
       return rc;
err_out0:       
       kfree(attr);
       return rc;
}
#endif

static int ram_bb_grow(struct ram_buffer *bb ,  int new_size )
{
        char * buf ;
        int order = ram_get_pageorder( new_size );
        int size = ( (1<<order)*PAGE_SIZE );
        int size_old = RAM_MAX_SIZE( bb );
        if( ram_get_total_size() - size_old +  size
                > RAM_MAX_BUFFER_SIZE || size > RAM_MAX_ONE_BUFFER_SIZE ) {
                return -ENOMEM;
         }
         
         buf = (char*)__get_free_pages(GFP_KERNEL , order );
         if( !buf ) 
                return -ENOMEM;
        memset( buf , 0 , size );
        pr_debug("%s:: file=%s grow from %dK to %dk \n" ,__func__ , bb->dest_filename,size_old>>10,size>>10);
        down( &mutex_ram_open );
        memcpy(buf , bb->buffer , bb->size );
        free_pages( (unsigned long)bb->buffer , bb->page_order );
        bb->buffer = buf;
        bb->page_order = order;
        total_ram_size += size - size_old;
        up( &mutex_ram_open );
        return 0;
}

#if 0
/**
 * wirte a config file , let init shell do the things.
 */
void ram_link_file_work(struct work_struct *work)
{
        struct ram_buffer *bb;
                
        down( &mutex_ram_open );
        list_for_each_entry( bb , &ram_opend_list , list ) {
                if( !(bb->flags&RAM_FL_CREATED) ) {
                                ram_create_file( bb );
                }
        }
        up( &mutex_ram_open );
}

DECLARE_DELAYED_WORK(ram_work, ram_link_file_work);	
#endif

static void ram_add_file_list( struct ram_buffer *bb )
{
        INIT_LIST_HEAD( &bb->list );
        bb->flags |= RAM_FL_FLUSHED;    /* XXX:RAM_FL_CREATED must be kept */
        down( &mutex_ram_open );
        list_add( &bb->list , &ram_opend_list);
        up( &mutex_ram_open );
}

static void ram_del_file_list( struct ram_buffer *bb )
{
        ram_unregister_file( bb->dest_filename );
        
        down( &mutex_ram_open );
        list_del( &bb->list );
        up( &mutex_ram_open );
        kfree( bb );
}

static inline int ram_check_file_type( char *filename , char* type )
{
        int len = strlen(filename)-strlen(type);
        if( len > 0 ) {
                if( !strcmp( filename+len , type ) )
                        return 1;
        }
        return 0;
}

int ram_register_file(struct file *filp , int reg  )
{
        int error;
        size_t total_size;
        int order;
        int size;
        struct ram_buffer *bb ;
        
        if( !ram_configed )
                        return -ENOTSUPP;
        /* 20091231,HSL@RK,ram funtion will crash nfs root. */
        if( strstr(saved_command_line,"nfsroot=") ) {
                ram_configed = 0;
                return -ENOTSUPP;
        }
        
        bb = ram_search_baking_file( filp->f_name , 1);
         if( bb ) 
                        goto found_out;
        if( !reg )
                        return -ENODEV;
                        
        if(  !ram_check_file_type(filp->f_name , ".db-journal") )
                return -ENOTSUPP;
                
        error = -ENODATA;
        total_size = ram_get_total_size();
        order = ram_get_pageorder( i_size_read( filp->f_path.dentry->d_inode ) );
        if( ram_check_file_type(filp->f_name,".db") ) /* db file *2 to db-journal */
                order++;
                
        pr_debug("%s::register %s,order=%d,total_size=%dK,mode=0%o\n" ,
                __func__ , filp->f_name , order , total_size>>10 , filp->f_save_mode);
        /*db-journal: O_RDWR|O_CREAT|O_EXCL|O_LARGEFILE|O_NOFOLLOW */
        size = PAGE_SIZE*(1<< order);
        if( size > RAM_MAX_ONE_BUFFER_SIZE || total_size+size > RAM_MAX_BUFFER_SIZE )
                goto err_out;
                
        error = -ENOMEM;
        bb = kzalloc(sizeof(*bb), GFP_KERNEL);
        if (!bb)
        	goto err_out;
        strcpy( bb->dest_filename, filp->f_name );
        ram_get_filename( bb );
        bb->page_order = order;
        
        ram_add_file_list( bb );
        
        /* delay work */
        //schedule_delayed_work( &ram_work , 1);
found_out:
        bb->filp = filp;
        get_task_comm(bb->proc_name , current );
        if( bb->mode != (filp->f_save_mode&0x3ff) ) {
                bb->mode = (filp->f_save_mode&0x3ff);
                bb->flags &= ~RAM_FL_LOADED ; /* need to reload */
        }
        ram_loader_file( bb );
        bb->write_count += filp->f_nr_writes;
        return 0;
err_out:
        return error;
}

int ram_unregister_file(char * filename )
{
        struct ram_buffer *bb = ram_search_baking_file( filename , 1);
        if( !bb )
                return -ENODEV;
        pr_debug("%s::unregister %s\n" , __func__ , filename );
        if( bb->filp ) {
                bb->filp->f_ramop = NULL;
                bb->filp->private_data = NULL;
                bb->filp = NULL;
        }
        bb->flags |= RAM_FL_FLUSHED;
        ram_free_buffer( bb );
        return 0;
}

static ssize_t sysram_read(struct file *file, char __user *userbuf,
		     size_t bytes, loff_t *off)
{
                struct ram_buffer *bb = file->private_data;
                loff_t size;
                loff_t offs = *off;
                size_t count = bytes;

                pr_debug("%s::off=0x%Lx,count=%d\n" , __func__ , offs , count);
                if( !bb || file != bb->filp || !bb->buffer ) {
                        return -EIO;
                        goto out_failed;
                }
                size = bb->size;
                if( !size || offs >= size )
                        return 0;
                if( offs + bytes > size ) {
                        count = size - offs ;
                }
                if( userbuf == NULL ) {
                        count = -EACCES;
                        goto out_failed;
                }
	down( &mutex_ram_open );
	if (copy_to_user( userbuf ,(const void*) bb->buffer+offs, count)) {
		count = -EFAULT;
		goto out_unlock;
	}
	*off = offs + count;
 out_unlock:
	up( &mutex_ram_open );
	return count;
out_failed:        
                printk("%s::file %s read error,len=0x%Lx,cur size=%d,ret=%d\n" ,
                        __func__ , file->f_name , offs + bytes , bb->size , count );
	return count;
}

static ssize_t sysram_write(struct file *file, const char __user *userbuf,
		     size_t bytes, loff_t *off)
{
                struct ram_buffer *bb = (struct ram_buffer*)file->private_data;
                loff_t msize ;
                loff_t offs = *off;
                size_t count = bytes;

                pr_debug("%s::off=0x%Lx,count=%d \n" , __func__ , offs , count );
                        
                if( !bb || file != bb->filp || !bb->buffer )
                        return -EIO;
                msize = RAM_MAX_SIZE(bb);
                if( offs + bytes > msize ) {
                        if( ram_bb_grow( bb , offs+bytes ) ) {
                                pr_debug("%s:: ram_bb_grow failed,file=%s\n" , __func__ , bb->dest_filename );
                                ram_save_file( bb );
                                ram_del_file_list(bb);
                                return -EAGAIN;
                        }
                        msize = RAM_MAX_SIZE(bb);
                }
              

                if( userbuf == NULL || count == 0 ) {
                        count = -EPERM;
                        goto out_failed;
                }
	down( &mutex_ram_open );
	if (copy_from_user(bb->buffer+offs, userbuf, count)) {
		count = -EFAULT;
		goto out_unlock;
	}
                bb->flags &= ~ (RAM_FL_FLUSHED);
                bb->mode &= ~ (O_TRUNC);
	*off = offs + count;
                if( *off > bb->size )
                        bb->size = *off;
 out_unlock:
	up( &mutex_ram_open );
	return count;
out_failed:  
                printk("%s::file %s write overload,need file size=0x%Lx,ret=%d\n" ,
                        __func__ , file->f_name , offs + bytes , count );
                BUG();
	return count;
}


int ram_init_file( char *dest_file , char *sys_file , 
        mode_t mod, int size )
{
        int error;
        size_t total_size;
        int order;
        struct ram_buffer *bb = ram_search_baking_file( sys_file , 0);
         if( bb ) 
                        goto found_out;
        error = -ENODATA;
        total_size = ram_get_total_size();
        order = ram_get_pageorder( size );
        if( total_size+PAGE_SIZE*(1<< order) > RAM_MAX_BUFFER_SIZE )
                goto err_out;
        error = -ENOMEM;
        bb = kzalloc(sizeof(*bb), GFP_KERNEL);
        if (!bb)
        	goto err_out;
        strcpy( bb->dest_filename, dest_file );
        strcpy( bb->sys_filename, sys_file );
        bb->mode = mod;
        bb->page_order = order;
        ram_add_file_list( bb );
        bb->flags |= RAM_FL_CONFLUSH;
        ram_loader_file( bb ) ;
        
found_out:
        return 0;
err_out:
        return error;
}
/*
 * format : "xxxxk mod dest_name sys_name\n"
 *
 *
 */
static int ram_build_bb( char * content )
{
        char *p ;
        char *q;
        int ksize;
        mode_t mod;
        char * filename[2];
        int index;
        while( (p = strchr( content , '\n' )) ) {
                ksize = simple_strtoul( content , &q , 0 );
                if( *q == 'k' || *q == 'K' ) {
                        ksize <<= 10;
                        q++;
                }
                if( *q  != ' ' )
                        return -EIO;
                q++;
                mod = simple_strtoul( q , &q , 0 );
                if( *q != ' ' )
                        return -EIO;
                q++;
                index = 0;
                content = q;
                while( q < p ) {
                        if( *q == ' ' ) {
                               filename[index++] =  content;
                               *q = '\0';
                               content = q+1;
                        }
                        q++;
                }
                if( index != 1 )
                        return -EIO;
                filename[index++] =  content;
                 *q = '\0';
                if( ram_init_file(filename[0],filename[1], mod , ksize) ) {
                        printk("%s:: build %s failed,ksize=%d,mod=0x%x\n" , __func__ , filename[0] ,
                                ksize , mod );
                        return -EIO;
                }
                content = p+1;
         }

          /* delay work */
        //schedule_delayed_work( &ram_work , 1);
        return 0;
}

static int ram_load_config( const char* config_file )
{
        ssize_t nread;
        loff_t			file_offset=0;
        unsigned int		amount = 0;
        mm_segment_t old_fs;
        struct file * filp;
        char *buf;

        ram_configed = 1;
        filp = filp_open( config_file , O_RDONLY|O_SYNC ,  0777);
        if (IS_ERR(filp)) {
	printk("%s::unable to open config file: %s\n",__func__ , config_file);
	return -ENOENT;
        }
        amount = i_size_read( filp->f_path.dentry->d_inode );
        if( amount > PAGE_SIZE )
            amount = PAGE_SIZE-1;
        if( amount > 0 ) {
                buf = kzalloc( amount , GFP_KERNEL);
                if( !buf )
                    goto out;
                old_fs = get_fs();
                set_fs(KERNEL_DS);
                nread = vfs_read(filp, buf ,amount, &file_offset); /* 1 for '\0' */
                set_fs(old_fs);
                if( nread > 0 ){
                        int r = ram_build_bb( buf );
                        pr_debug("%s::read config file: %s OK, nread=%d,build=%d\n",__func__ , config_file , nread , r );
                        
                }
                kfree( buf );
        }
        fput(filp);
        return 0;
out:
        fput(filp);
        return -EIO;    
}
const struct file_operations ram_fops = {
	.read		= sysram_read,
	.write		= sysram_write,
	.llseek		= generic_file_llseek,
	.open		= ram_open,
	.release	= ram_release,
};

const struct file_operations ram_rw_fops = {
	.read		= sysram_read,
	.write		= sysram_write,
};


int ram_notifier_call(struct notifier_block *nfb , unsigned long code , void * cmd )
{
        if (code==SYS_RESTART || code==SYS_POWER_OFF ) {
                pr_debug("%s::gona to do sysfs ram sync.\n",__func__ );
                ram_sync_config();
                ram_list_sync( -1 );
        }
        return NOTIFY_DONE;
}

struct notifier_block sys_ram_notifier= {
        .notifier_call = ram_notifier_call,
};

/**
 *  load config file to build map info.
 * call at shell .
 *
 */
int ram_sys_init( char *config_path )
{
        int ret = register_reboot_notifier( &sys_ram_notifier );
        if (ret != 0) {
        	printk(KERN_ERR "cannot register reboot notifier (err=%d)\n",ret);
        }
        if( config_path )
                strcpy( default_config_filepath , config_path );
        ram_load_config( default_config_filepath );
        return ret;
}

#else

int ram_register_file(struct file *filp , int reg  )
{       
        return -EIO;
}
int ram_unregister_file(char * filename )
{
        return -EIO;
}
int ram_sync( void )
{       
        return -EIO;
}

const struct file_operations ram_fops ;
#endif

