#ifndef _TOUCHSCREEN_H
#define _TOUCHSCREEN_H

typedef enum
{
	ID_MAIN	=0,
	ID_SUB		=1,
	ID_INVALID	=2,
}touch_id_type;

typedef struct touchscreen_funcs {
 touch_id_type touch_id;
 int (*active) (void ); 
 int (*get_wakeup_gesture)(char *); 
 int (*get_gesture_ctrl)(char *);
 int (*gesture_ctrl)(const char *); 
}touchscreen_ops_tpye;


#endif /* _TOUCHSCREEN_H */


extern int touchscreen_set_ops(touchscreen_ops_tpye *ops);

