#ifndef H_MPU9250_H
#define H_MPU9250_H

#include <stdint.h>

/*!
 *	Register adreslerinin ve bazi sabit verilerin bilgilerini tutan makro tanimlamalari
 */
#define MPU9250_ADRESS                    (0x68)                                       /*!< The address of the the devices should be b1101000 (pin AD0 is logic low) */
#define MPU9250_WHO_AM_I                  (0x75)                                       /*!< The default value of the register is 0x71                                */
#define MPU9250_WHO_AM_I_VALUE			  (0x71)									   /*!< Who Am I register default value											 */ 
#define MPU9250_PWR_MGMT_1                (0x6B)                                       /*!< Power Management 1 Register                                              */
#define MPU9250_PWR_MGMT_2                (0x6C)                                       /*!< Power Management 2 Register                                              */
#define MPU9250_CONFIG					  (0X1A)									   /*!< Configuration Register													 */
#define MPU9250_GYRO_CONFIG               (0x1B)                                       /*!< Gyro Configuration Register                                              */
#define MPU9250_ACCEL_CONFIG              (0x1C)                                       /*!< Accelerometer Configuration                                              */
#define MPU9250_ACCEL_CONFIG_2			  (0X1D)									   /*!< Accelerometer Configuration  2											 */
#define MPU9250_ACCEL_XOUT_H              (0x3B)                                       /*!< Accelerometer X-Out High Register                                        */
#define MPU9250_ACCEL_XOUT_L              (0x3C)                                       /*!< Accelerometer X-Out Low  Register                                        */
#define MPU9250_ACCEL_YOUT_H              (0x3D)                                       /*!< Accelerometer Y-Out High Register                                        */
#define MPU9250_ACCEL_YOUT_L              (0x3E)                                       /*!< Accelerometer Y-Out Low  Register                                        */
#define MPU9250_ACCEL_ZOUT_H              (0x3F)                                       /*!< Accelerometer Z-Out High Register                                        */
#define MPU9250_ACCEL_ZOUT_L              (0x40)                                       /*!< Accelerometer Z-Out Low  Register                                        */
#define MPU9250_TEMP_OUT_H                (0x41)                                       /*!< Temperature High Register                                                */
#define MPU9250_TEMP_OUT_L                (0x42)                                       /*!< Temperature Low  Register                                                */
#define MPU9250_GYRO_XOUT_H               (0x43)                                       /*!< Gyroscope X-Out High Register                                            */
#define MPU9250_GYRO_XOUT_L               (0x44)                                       /*!< Gyroscope X-Out Low  Register                                            */
#define MPU9250_GYRO_YOUT_H               (0x45)                                       /*!< Gyroscope Y-Out High Register                                            */
#define MPU9250_GYRO_YOUT_L               (0x46)                                       /*!< Gyroscope Y-Out Low  Register                                            */
#define MPU9250_GYRO_ZOUT_H               (0x47)                                       /*!< Gyroscope Z-Out High Register                                            */
#define MPU9250_GYRO_ZOUT_L               (0x48)                                       /*!< Gyroscope Z-Out Low  Register                                            */

#define MPU9250_GYRO_SENSIVITY_250DPS     (131)                                        /*!< Gyro sensivity scale factor FS_SEL = 0, 131 LSB/(�/s)   				*/
#define MPU9250_GYRO_SENSIVITY_500DPS     (63.5F)                                      /*!< Gyro sensivity scale factor FS_SEL = 1, 63.5 LSB/(�/s) 					*/
#define MPU9250_GYRO_SENSIVITY_1000DPS    (32.8F)                                      /*!< Gyro sensivity scale factor FS_SEL = 2, 23.8 LSB/(�/s) 					*/
#define MPU9250_GYRO_SENSIVITY_2000DPS    (16.4F)                                      /*!< Gyro sensivity scale factor FS_SEL = 3, 16.4 LSB/(�/s)                  */
#define MPU9250_ACCEL_SENSIVITY_2G        (16384.0F)                                   /*!< Accel sensivity scale factor AFS_SEL = 0, 16,384 LSB/g                   */
#define MPU9250_ACCEL_SENSIVITY_4G        (8192.0F)                                    /*!< Accel sensivity scale factor AFS_SEL = 0, 8.192 LSB/g                    */
#define MPU9250_ACCEL_SENSIVITY_8G        (4096.0F)                                    /*!< Accel sensivity scale factor AFS_SEL = 0, 4.096F LSB/g                   */
#define MPU9250_ACCEL_SENSIVITY_16G       (2048.0F)                                    /*!< Accel sensivity scale factor AFS_SEL = 0, 2.048F LSB/g                   */
#define SENSORS_RADS_TO_DPS               (57.295779513082320876798154814105F)         /*!< From deg to rad conversion                                               */
#define EARTH_GRAVITY                     (9.800665F)                                  /*!< Earth's Gravity force value                                              */
#define ALPHA 							  (0.05000F)								   /*!< Complementart filter rate 												 */								  							
#define LOW_PASS_FILTER_RATE			  (0.90F)
#define SMOOTH_FILTER_SAMPLES 			  (21)										   /*! must be > 3 and one-digit   			 									 */			

/*!
 * Client kodlar yapinin elemanlarini degistirmemelidir 
 * Elemanlara yalnizca okuma amacli erisilmeli ve bu islem
 * getter fonksiyonlari ile yapilmalidir.
 */
typedef struct									
{
	int16_t  	 	ax;					/*!< Accel x register value  	*/
	int16_t  	 	ay;					/*!< Accel y register value  	*/
	int16_t  	 	az;					/*!< Accel z register value  	*/
	int16_t  	 	gx;					/*!< Gyro x register value	 	*/
	int16_t  	 	gy;					/*!< Gyro y	register value	 	*/
	int16_t  	 	gz;					/*!< Gyro z	register value	 	*/
	int16_t  	 	t;					/*!< Temp register value     	*/
	double 			axc;				/*!< Calibration accel x value	*/
	double			ayc;				/*!< Calibration accel y value	*/
	double			azc;				/*!< Calibration accel z value	*/
	double			gxc;				/*!< Calibration gyro  x value	*/
	double			gyc;				/*!< Calibration gyro  y value	*/
	double			gzc;				/*!< Calibration gyro  z value	*/
	double 	 	 	deg_ax;				/*!< Degree accel x	value		*/
	double 	 	 	deg_ay;				/*!< Degree accel y	value		*/
	double 	 	 	deg_az;				/*!< Degree accel z	value		*/
	double 	 	 	deg_gx;				/*!< Degree gyro  x	value		*/
	double 	 	 	deg_gy;				/*!< Degree gyro  y	value		*/
	double 	 	 	deg_gz;				/*!< Degree gyro  z	value		*/
	double 			last_pitch;			
	double 			last_roll;
	double 			last_deg_gx;
	double 			last_deg_gy;
	double 	 	 	elapsed_time;		
	unsigned int 	current_time;
	unsigned int 	previous_time;		
	double 			pitch;
	double 			roll;
	double 			temp;

} MPU9250_TypeDef;

/*!
 * Utility Function Prototypes 
 */
int MPU9250_WhoAmI(void);												/*!< Dogru sensor mu konrol edilir 						   	*/
void lowPassFilter(MPU9250_TypeDef* preg);								
double digitalSmooth(double rawIn, double *sensSmoothArray);			/*!< Bir sensorden gelen veriler yumusatilir				*/
MPU9250_TypeDef* calibration_accel(MPU9250_TypeDef* preg);				/*!< Accelerometer kalibrasyonu yapilir					   	*/
MPU9250_TypeDef* calibration_gyro(MPU9250_TypeDef* preg); 				/*!< Gyroscope kalibrasyonu yapilir					   		*/
MPU9250_TypeDef* complementary_filter(MPU9250_TypeDef* preg);			/*!< Accel ve Gyro verileri ile pitch-roll elde edilir	   	*/

/*!
 * Set Functions Prototypes 
 */		
void MPU9250_init(void);												/*!< Baslangic ayarlari yapilir 						   	*/
MPU9250_TypeDef* set_reg_accel(MPU9250_TypeDef* preg);					/*!< Accelerometer degerleri set edilir 				   	*/
MPU9250_TypeDef* set_reg_gyro(MPU9250_TypeDef* preg);					/*!< Gyroscope degerleri set edilir 					   	*/
MPU9250_TypeDef* set_reg_temp(MPU9250_TypeDef* preg);					/*!< Temperature degerleri set edilir   				   	*/
MPU9250_TypeDef* set_accel_deg_xyz(MPU9250_TypeDef* reg);				/*!< Accelerometer aci degerleri set edilir				   	*/	
MPU9250_TypeDef* set_gyro_deg_xyz(MPU9250_TypeDef* reg);				/*!< Gyroscope aci degerleri set edilir				   	   	*/
MPU9250_TypeDef* set_temp(MPU9250_TypeDef* preg);						/*!< Temperature verisi set edilir						   	*/
void set_pitch(double pitch,MPU9250_TypeDef* preg);
void set_roll(double roll, MPU9250_TypeDef* preg);

/*!
 * Get Functions Prototypes 
 */
int16_t get_accel_raw_x(const MPU9250_TypeDef* preg);
int16_t get_accel_raw_y(const MPU9250_TypeDef* preg);
int16_t get_accel_raw_z(const MPU9250_TypeDef* preg);
int16_t get_temp_raw(const MPU9250_TypeDef* preg);
int16_t get_gyro_raw_x(const MPU9250_TypeDef* preg);
int16_t get_gyro_raw_y(const MPU9250_TypeDef* preg);
int16_t get_gyro_raw_z(const MPU9250_TypeDef* preg);
double  get_accel_cal_x(const MPU9250_TypeDef* preg);
double  get_accel_cal_y(const MPU9250_TypeDef* preg);
double  get_accel_cal_z(const MPU9250_TypeDef* preg);
double  get_gyro_cal_x(const MPU9250_TypeDef* preg);
double  get_gyro_cal_y(const MPU9250_TypeDef* preg);
double  get_gyro_cal_z(const MPU9250_TypeDef* preg);
double 	get_accel_deg_x(const MPU9250_TypeDef* preg);
double 	get_accel_deg_y(const MPU9250_TypeDef* preg);
double 	get_accel_deg_z(const MPU9250_TypeDef* preg);
double 	get_gyro_deg_x(const MPU9250_TypeDef* preg);
double 	get_gyro_deg_y(const MPU9250_TypeDef* preg);
double 	get_gyro_deg_z(const MPU9250_TypeDef* preg);
double  get_pitch(const MPU9250_TypeDef* preg);
double  get_roll(const MPU9250_TypeDef* preg);
double 	get_temp(const MPU9250_TypeDef* preg);

#endif