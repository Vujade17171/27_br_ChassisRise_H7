#ifndef __RETURN_TYPES_H
#define __RETURN_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 统一返回码（全层通用） =====================
 * 约定：0 = 成功；负 = 错误。
 * 两套语义正好相反，跨层混用极易出错 —— 本工程统一为下面这一套。 */
#define SER_OK           0
#define SER_ERR_PARAM  (-1)   /* 空指针 / 参数越界                  */
#define SER_ERR_MODEL  (-2)   /* 电机量程模型不合法                 */
#define SER_ERR_BUS    (-3)   /* 总线发送失败（队列满 / 未初始化）   */
#define SER_ERR_LIMIT  (-4)   /* 超机械限位 / 目标不可达 / 参数非法  */
#define SER_ERR_STATE  (-5)   /* 当前状态不允许该操作               */
#define SER_ERR_NOENT  (-6)   /* 未找到匹配对象（ID 不匹配等）       */

#ifdef __cplusplus
}
#endif

#endif /* __RETURN_TYPES_H */
