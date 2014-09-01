/*******************************************************************************
 *   Description:   TODO
 *
 ******************************************************************************/

#ifndef _TN_MEM_H
#define _TN_MEM_H

/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/

#include "tn_utils.h"
#include "tn_common.h"



/*******************************************************************************
 *    PUBLIC TYPES
 ******************************************************************************/

struct TN_Fmp {
   struct TN_QueHead wait_queue;

   unsigned int block_size; //-- Actual block size (in bytes)
   int num_blocks;          //-- Capacity (Fixed-sized blocks actual max qty)
   void *start_addr;        //-- Memory pool actual start address
   void *free_list;         //-- Ptr to free block list
   int fblkcnt;             //-- Num of free blocks
   enum TN_ObjId id_fmp;              //-- ID for verification(is it a fixed-sized blocks memory pool or another object?)
                            // All Fixed-sized blocks memory pool have the same id_fmp magic number (ver 2.x)
};

/*******************************************************************************
 *    GLOBAL VARIABLES
 ******************************************************************************/

/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 *    PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

int tn_fmem_create(struct TN_Fmp  * fmp,
      void * start_addr,
      unsigned int block_size,
      int num_blocks);
int tn_fmem_delete(struct TN_Fmp * fmp);
int tn_fmem_get(struct TN_Fmp * fmp, void ** p_data, unsigned long timeout);
int tn_fmem_get_polling(struct TN_Fmp * fmp, void ** p_data);
int tn_fmem_get_ipolling(struct TN_Fmp * fmp, void ** p_data);
int tn_fmem_release(struct TN_Fmp * fmp, void * p_data);
int tn_fmem_irelease(struct TN_Fmp * fmp, void * p_data);


#endif // _TN_MEM_H

/*******************************************************************************
 *    end of file
 ******************************************************************************/

