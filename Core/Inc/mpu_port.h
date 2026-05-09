#ifndef __MPU_PORT_H
#define __MPU_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Forward declaration - defined in inv_mpu.h */
struct int_param_s;

int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data);
void delay_ms(unsigned long num_ms);
void get_ms(unsigned long *count);
void __no_operation(void);
int min(int a, int b);
void reg_int_cb(struct int_param_s *param);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);

#ifdef __cplusplus
}
#endif

#endif /* __MPU_PORT_H */
