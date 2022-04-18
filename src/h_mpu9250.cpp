#include "h_mpu9250.h"
#include "Arduino.h"
#include <Wire.h>
#include <math.h>

#define PUBLIC		
#define PRIVATE				static
#define SAMPLE_SIZE			200				/*! Konfigurasyon yaparken toplam yapilacak ornekleme sayisidir istege bagli degistirilebilir      */
#define GYRO_SENSIVITY			MPU9250_GYRO_SENSIVITY_250DPS	/*! Eger sensivity degistirilecekse buraya istenen makro yazilmali ve init fonksiyonu duzenlenmeli */
#define ACCEL_SENSIVITY 		MPU9250_ACCEL_SENSIVITY_2G	/*! Eger sensivity degistirilecekse buraya istenen makro yazilmali ve init fonksiyonu duzenlenmeli */

/*!< Static functions prototypes */
PRIVATE void MPU9250_requestBytes(uint8_t address, uint8_t subAddress, uint8_t bytes);
PRIVATE void MPU9250_read_to_array(uint8_t* output, size_t size);
PRIVATE int16_t MPU9250_uint8ToUint16(uint8_t Lbyte, uint8_t Hbyte);
PRIVATE void i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t byteValue);
PRIVATE uint8_t i2c_read8RegisterByte(uint8_t deviceAddress, uint8_t registerAddress);

/*!
 *  @brief  Sensorun varliginin kontrol eder.  
 *          Who Am I register'i icerisinde ki default deger 0x71 olmalidir. 
 *  
 *  @return
 *	    Basari durumunda '1'
 *	    Basarisizlik durumunda '0'
 */
PUBLIC int MPU9250_WhoAmI(void)
{
    uint8_t buff = i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_WHO_AM_I);
    if(buff == MPU9250_WHO_AM_I_VALUE)
	return 1;
		
    return 0;
}

/*!
 * @brief   Sensorun calisabilmesi icin baslangic ayarlari yapilir
 */
PUBLIC void MPU9250_init(void)
{
    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1, 0x80);                    /*!< Reset the device                                                         */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1);                          /*!< Wait until it's written                                                  */

    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1, 0x00);                    /*!< Wake up                                                                  */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1);                          /*!< Wait until it's written                                                  */

    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1, 0x01);                    /*!< Auto selects the best available clock source                             */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_PWR_MGMT_1);                          /*!< Wait until it's written                                                  */
    
    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_GYRO_CONFIG, 0x00);               	/*!< Gyro Full Scale Select: 00 = +250 dps  Fchoice_b[1:0] = 0xb00            */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_GYRO_CONFIG);                         /*!< Wait until it's written                                                  */

    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_ACCEL_CONFIG, 0x00);                  /*!< Accel Full Scale Select: 00 = +-2 g                                      */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_ACCEL_CONFIG);                        /*!< Wait until it's written                                                  */

    i2c_writeRegisterByte(MPU9250_ADRESS, MPU9250_ACCEL_CONFIG_2, 0x08);           	/*!< Accel BW = 460 Hz Rate = 1 kHz                                           */
    i2c_read8RegisterByte(MPU9250_ADRESS, MPU9250_ACCEL_CONFIG_2);                      /*!< Wait until it's written 						      */
}

/*!
 * @brief   Gyro ve accel verileri birlestirilerek pitch-roll verileri elde edilir
 */
PUBLIC MPU9250_TypeDef* complementary_filter(MPU9250_TypeDef* preg)
{
    preg->roll = (1.0f - ALPHA) * (preg->deg_gx + preg->last_roll) + ALPHA * preg->deg_ax;
    preg->pitch = (1.0f - ALPHA) * (preg->deg_gy + preg->last_pitch) ALPHA * preg->deg_ay;

    preg->last_roll = preg->roll;
    preg->last_pitch = preg->pitch;
	
    return preg;
}

/*!
 * @brief   Accelerometer raw degerleri set edilir
 */
PUBLIC MPU9250_TypeDef* set_reg_accel(MPU9250_TypeDef* preg)
{
    MPU9250_requestBytes(MPU9250_ADRESS, MPU9250_ACCEL_XOUT_H, 6);

    uint8_t raw_data[6] = { 0 };
    MPU9250_read_to_array(raw_data, 6);

    preg->ax = MPU9250_uint8ToUint16(raw_data[1], raw_data[0]);
    preg->ay = MPU9250_uint8ToUint16(raw_data[3], raw_data[2]);
    preg->az = MPU9250_uint8ToUint16(raw_data[5], raw_data[4]);
	
    return preg;
}

/*!
 * @brief  Gyroscope raw degerleri set edilir
 *
 *         previous_time   = bir onceki zaman
 * 	   current_time    = suan ki zaman 
 *	   elapsed_time	   = saniye cinsinden onceki ve simdi ki zamanin farki
 */
PUBLIC MPU9250_TypeDef* set_reg_gyro(MPU9250_TypeDef* preg)
{   
    preg->previous_time = preg->current_time;                 
    preg->current_time = millis();                            
    preg->elapsed_time = (double)(preg->current_time - preg->previous_time) / 1000.0f; // milisecond to second
	
    MPU9250_requestBytes(MPU9250_ADRESS, MPU9250_GYRO_XOUT_H, 6);

    uint8_t raw_data[6] = { 0 };
    MPU9250_read_to_array(raw_data, 6);

    preg->gx = MPU9250_uint8ToUint16(raw_data[1], raw_data[0]);
    preg->gy = MPU9250_uint8ToUint16(raw_data[3], raw_data[2]);
    preg->gz = MPU9250_uint8ToUint16(raw_data[5], raw_data[4]);
	
    return preg;
}

/*!
 * @brief Temperature raw degerleri set edilir
 */
PUBLIC MPU9250_TypeDef* set_reg_temp(MPU9250_TypeDef* preg)
{
    MPU9250_requestBytes(MPU9250_ADRESS, MPU9250_TEMP_OUT_H, 2);

    uint8_t raw_data[2] = { 0 };
    MPU9250_read_to_array(raw_data, 2);

    preg->t = MPU9250_uint8ToUint16(raw_data[1], raw_data[0]);
	
    return preg;
}


/*!
 * @brief   Ilk dongu icerisinde register'lar uzerinden alinan ham veri, ayarlarda
 *          secilen hassaslik degerine bolunur ve dunyanin yer cekimi ivmesi ile
 *          carpilir. X,Y,Z acilarini elde edebilmek icin kullanilacak olan square
 *          degeri ( X^2 + Y^2 + Z^2 ) toplaminin karekoku ile bulunur o sebeple toplam
 *          hesaplanir. Bir sonraki dongude ise arcsin alinir ve elde edilen deger
 *          radyandan dereceye donusturulur sonuc olarak tum acilar elde edilir.
 */
PUBLIC MPU9250_TypeDef* set_accel_deg_xyz(MPU9250_TypeDef* preg)
{
    double accel_scale = ACCEL_SENSIVITY;
    double accel_xyz[3] = {0};

    accel_xyz[0] = ((double)preg->ax - preg->axc) / accel_scale;
    accel_xyz[1] = ((double)preg->ay - preg->ayc) / accel_scale;
    accel_xyz[2] = ((double)preg->az - preg->azc) / accel_scale;

    double square_sum = 0;
    for (int i = 0; i < 3; ++i)
        square_sum += pow(accel_xyz[i], 2);

    double square_root = sqrt(square_sum);    

    preg->deg_ax = asinf((double)accel_xyz[0] / square_root) * RAD_TO_DEG;
    preg->deg_ay = atanf((double)accel_xyz[1] / accel_xyz[2]) * RAD_TO_DEG;

    return preg;
}

/*!
 * @brief  Derece cinsinden gyroscope'un x-y-z verileri elde edilir.
 *
 *         gyro_scale 		= gyroscope olceklemesi
 *         elapsed_time 	= loop icerisinde bir gyroscope verisi 
 *				  okunana kadar gecen sure
 *         deg_(x-y-z)  	= derece cinsinden veriler
 * 	   gxc, gyc, gzc	= gyroscope kalibrasyon degerleri
 */
PUBLIC MPU9250_TypeDef* set_gyro_deg_xyz(MPU9250_TypeDef* preg)
{
    double period = 1 / preg->elapsed_time;

    preg->deg_gx = ((double)preg->gx - preg->gxc) / GYRO_SENSIVITY / period;
    preg->deg_gy = ((double)preg->gy - preg->gyc) / GYRO_SENSIVITY / period;
    preg->deg_gz = ((double)preg->gz - preg->gzc) / GYRO_SENSIVITY / period;

    return preg;
}

/*!
 * @brief   Santigrat cinsinden sicaklik elde edilir  
 *
 *	    temp_scale  = temperature sensivity LSB/C
 *	    temp_offset = room temp offset LSB
 */
PUBLIC MPU9250_TypeDef* set_temp(MPU9250_TypeDef* preg)
{
    set_reg_temp(preg);
	
    static const float temp_scale 	= 333.87f;
    static const float temp_offset 	= 21.0f;

    preg->temp = ((((double)preg->t) - temp_offset) / temp_scale) + temp_offset;
	
    return preg;
}

PUBLIC void set_pitch(double pitch, MPU9250_TypeDef* preg)
{
    preg->pitch = pitch;
}

PUBLIC void set_roll(double roll, MPU9250_TypeDef* preg)
{
    preg->roll = roll;
}

/*!
 * @brief   Accelerometer offset ayari yapilir  
 *          
 *          axs = accelerometer x simple 
 * 	    ays = accelerometer y simple
 * 	    azs = accelerometer z simple
 */
PUBLIC MPU9250_TypeDef* calibration_accel(MPU9250_TypeDef* preg)
{
    double axs = 0;
    double ays = 0;
    double azs = 0;
	
    for(size_t i = 0; i < SAMPLE_SIZE; ++i) {
	set_reg_accel(preg);
	axs = ((double)preg->ax + axs) / (double)SAMPLE_SIZE;
	ays = ((double)preg->ay + ays) / (double)SAMPLE_SIZE;
	azs = ((double)preg->az + azs) / (double)SAMPLE_SIZE;
	delay(20);
    }
	
    preg->axc = axs;
    preg->ayc = ays;
    preg->azc = azs;
	
    return preg;
}

/*!
 * @brief  Gyroscope offset ayari yapilir  
 *
 *	   gxs = gyroscope x simple 
 *	   gys = gyroscope y simple
 *	   gzs = gyroscope z simple 
 */
PUBLIC MPU9250_TypeDef* calibration_gyro(MPU9250_TypeDef* preg)
{
    double gxs = 0;
    double gys = 0;
    double gzs = 0;
	
    for(size_t i = 0; i < SAMPLE_SIZE; ++i) {
	set_reg_gyro(preg);	
	gxs = ((double)preg->gx + gxs) / (double)SAMPLE_SIZE;
        gys = ((double)preg->gy + gys) / (double)SAMPLE_SIZE;
	gzs = ((double)preg->gz + gzs) / (double)SAMPLE_SIZE;
	delay(20);
    }
	
    preg->gxc = gxs;
    preg->gyc = gys;
    preg->gzc = gzs;
	
    return preg;
}

/*!
 * @brief   Dijital bir alcak geciren filtredir. 
 *          Anlik degisimlere karsi duyarliligini azaltir.
 *          Bunu yaparken bir onceki deger ile suan ki degerin 
 *          ciktilarini alarak belirlenen bir orana gore cikti verir.
 *
 *	    pre_a = previous accelerometer data
 */
PUBLIC void lowPassFilter(MPU9250_TypeDef* preg)
{
    static int16_t pre_a[3] = {0};
	
    preg->ax = pre_a[0] * (LOW_PASS_FILTER_RATE) + preg->ax * (1 - LOW_PASS_FILTER_RATE);
    preg->ay = pre_a[1] * (LOW_PASS_FILTER_RATE) + preg->ay * (1 - LOW_PASS_FILTER_RATE);
    preg->az = pre_a[2] * (LOW_PASS_FILTER_RATE) + preg->az * (1 - LOW_PASS_FILTER_RATE);
	
    pre_a[0] = preg->ax;
    pre_a[1] = preg->ay;
    pre_a[2] = preg->az;
}

/*!
 * 
 * @brief 
 *          Parametre olarak her sensorun kendisine ait olacak bir degerler dizisinin adresi verilir
 *          Yeni veriler eski veriler ile yer degistirir ve baska bir diziye aktarilir
 *          Degerler kucukten buyuge siralanir, en kucuk %15 ile en buyuk %15 diziden atilir ve 
 *          geriye kalan degerlerin ortalamasi alinarak cikisa iletilir 
 *
 * @param
 *          sensSmoothArray = raw sensor values array
 *          rawIn           = new raw sensor value
 * @return 
 *          smoothing signal 
 */
PUBLIC double digitalSmooth(double rawIn, double *sensSmoothArray)
{     
    static int i;  
    static double sorted[SMOOTH_FILTER_SAMPLES];
    
    i = (i + 1) % SMOOTH_FILTER_SAMPLES;    
    sensSmoothArray[i] = rawIn;                 
    
    for (size_t j = 0; j < SMOOTH_FILTER_SAMPLES; j++) {     
        sorted[j] = sensSmoothArray[j];
    }
    
    boolean done = 0;                
    while(done != 1) {       
        done = 1;
        for (size_t j = 0; j < (SMOOTH_FILTER_SAMPLES - 1); j++) {
            if (sorted[j] > sorted[j + 1]) {
                int temp = sorted[j + 1];
                sorted [j+1] =  sorted[j] ;
                sorted [j] = temp;
                done = 0;
            }
        }
    }
    
    int bottom = max(((SMOOTH_FILTER_SAMPLES * 15)  / 100), 1); 
    int top = min((((SMOOTH_FILTER_SAMPLES * 85) / 100) + 1  ), (SMOOTH_FILTER_SAMPLES - 1));   
    
    int k = 0;
    long double total = 0;
    for (int j = bottom; j < top; j++) {
        total += sorted[j];  
        k++; 
    }
    
    return (double)total / k;    
}

/*!
 * @brief   I2C ile bir adresten istenilen adette veri okunur.
 *
 * @param
 * 	    address, registerindan deger okunacak cihazin i2c adresi
 * 	    subAddress, okunmak istenen registerin adresi
 * 	    bytes, kac byte okuma gerceklestirilecegini bilgisi
 */
PRIVATE void MPU9250_requestBytes(uint8_t address, uint8_t subAddress, uint8_t bytes)
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, bytes);
}

/*!
 * @brief   I2C hattinda istenilen adet kadar okuma gerceklestirir.
 *
 * @param 
 *          output, set edilecek dizinin adresi
 *          size, dizinin boyutu
 */
PRIVATE void MPU9250_read_to_array(uint8_t* output, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        output[i] = (uint8_t)Wire.read();
    }
}

/*!
 * @brief   8 bitlik iki isaretsiz tam sayiyi 16 bitlik sayiya cevirir.
 *	
 * @return
 *          Yapilan islem sonucu elde edilen deger
 */
PRIVATE int16_t MPU9250_uint8ToUint16(uint8_t Lbyte, uint8_t Hbyte)
{
    return ((int16_t)Hbyte << 8) | Lbyte;
}

/*!
 *  Secilen registera 8 bitlik bir yazma gerceklestirir
 */
PRIVATE void i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t byteValue)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)registerAddress);
    Wire.write((uint8_t)byteValue);
    Wire.endTransmission();
}

/*!
 *  @brief  Secilen registerdan 8 bitlik bir okuma gerceklestirir
 * 
 *  @return
 *	    Basari durumunda okunan deger
 * 	    Basarisizlik durumunda '0'
 */
PRIVATE uint8_t i2c_read8RegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
{
    uint8_t data_value;

    Wire.beginTransmission(deviceAddress);
    Wire.write((uint8_t)registerAddress);
    uint8_t control = Wire.endTransmission();
    if (!control)
    {
        Wire.requestFrom(deviceAddress, (uint8_t)1);
        data_value = Wire.read();
        return data_value;
    }

    return 0;
}

/*! 
 * Client kodlarin nesnelere erisimini saglamak icin yazilan getter fonksiyonlari
 */
PUBLIC int16_t get_accel_raw_x(const MPU9250_TypeDef* preg)
{
    return preg->ax;
}

PUBLIC int16_t get_accel_raw_y(const MPU9250_TypeDef* preg)
{
    return preg->ay;
}
PUBLIC int16_t get_accel_raw_z(const MPU9250_TypeDef* preg)
{
    return preg->az;
}

PUBLIC int16_t get_temp_raw(const MPU9250_TypeDef* preg)
{
    return preg->t;
}

PUBLIC int16_t get_gyro_raw_x(const MPU9250_TypeDef* preg)
{
    return preg->gx;
}

PUBLIC int16_t get_gyro_raw_y(const MPU9250_TypeDef* preg)
{
    return preg->gy;
}

PUBLIC int16_t get_gyro_raw_z(const MPU9250_TypeDef* preg)
{
    return preg->gz;;
}

PUBLIC double get_accel_deg_x(const MPU9250_TypeDef* preg)
{
    return preg->deg_ax;
}

PUBLIC double get_accel_deg_y(const MPU9250_TypeDef* preg)
{
    return preg->deg_ay;
}

PUBLIC double get_accel_deg_z(const MPU9250_TypeDef* preg)
{
    return preg->deg_az;
}

PUBLIC double get_gyro_deg_x(const MPU9250_TypeDef* preg)
{
    return preg->deg_gx;
}

PUBLIC double get_gyro_deg_y(const MPU9250_TypeDef* preg)
{
    return preg->deg_gy;
}

PUBLIC double get_gyro_deg_z(const MPU9250_TypeDef* preg)
{
    return preg->deg_gz;
}

PUBLIC double get_pitch(const MPU9250_TypeDef* preg)
{
    return preg->pitch;
}

PUBLIC double get_roll(const MPU9250_TypeDef* preg)
{
    return preg->roll;
}

PUBLIC double get_temp(const MPU9250_TypeDef* preg)
{
    return preg->temp;
}

PUBLIC double  get_accel_cal_x(const MPU9250_TypeDef* preg)
{
    return preg->axc;
}

PUBLIC double  get_accel_cal_y(const MPU9250_TypeDef* preg)
{
    return preg->ayc;
}

PUBLIC double  get_accel_cal_z(const MPU9250_TypeDef* preg)
{
    return preg->azc;
}

PUBLIC double  get_gyro_cal_x(const MPU9250_TypeDef* preg)
{
    return preg->gxc;
}

PUBLIC double  get_gyro_cal_y(const MPU9250_TypeDef* preg)
{
    return preg->gyc;
}

PUBLIC double  get_gyro_cal_z(const MPU9250_TypeDef* preg)
{
    return preg->gzc;
}
