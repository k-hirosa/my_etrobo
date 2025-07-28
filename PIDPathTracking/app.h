#ifdef __cplusplus
extern "C" {
#endif

#include "spikeapi.h"

/* タスク優先度 */
#define MAIN_PRIORITY    (TMIN_APP_TPRI + 1) /* メインタスク */
#define TIMER_INTERRUPT_PRIORITY  (TMIN_APP_TPRI + 3) /* ライントレースタスク */
#define INIT_TASK_PRIORITY  (TMIN_APP_TPRI + 2) /* 初期化タスク */

/* タスク周期の定義 */
#define TIMER_INTERRUPT_PERIOD  (10 * 1000) /* ライントレースタスク:10msec周期 */
#define INIT_TASK_PERIOD  (10 * 1000) /* 初期化タスク:10msec周期 */


#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void timer_interrupt_task(intptr_t exinf);
extern void init_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif