 * \file
 *
 * Example project that demonstrates usage of queues in TNeo.
 */

#ifndef _TASK_CONSUMER_H
#define _TASK_CONSUMER_H

/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/

/*******************************************************************************
 *    PUBLIC TYPES
 ******************************************************************************/

enum E_TaskConsCmd {
  TASK_CONS_CMD__PIN_TOGGLE,
};

enum E_TaskConsPin {
  TASK_CONS_PIN__0,
  TASK_CONS_PIN__1,
  TASK_CONS_PIN__2,
};

void task_consumer_create(void);

int task_consumer_msg_send(enum E_TaskConsCmd cmd, enum E_TaskConsPin pin_num);

#endif  // _TASK_CONSUMER_H

/*******************************************************************************
 *    end of file
 ******************************************************************************/
