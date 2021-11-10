/*!< Verilerin seri ekranda izlenebilmesi icin SERIAL makrosunu define ediniz */
// #define SERIAL

/*!< Eklenen kutuphaneler       */
#include <Arduino.h>
#include <h_mpu9250.h>
#include <Wire.h>
#include <math.h>

/*! 
 * Smooth filtresinin default 21 olan ornekleme sayisini degistirmek istiyorsaniz 
 * yorum satirinda olan sirali #undef ve #define onislemci komutlarini yorum
 * satiri olmaktan cikarin ve 21 yazili olan 3'ten kucuk ve tek sayi olmak sarti
 * ile istediginiz degeri giriniz
 */
// #undef  SMOOTH_FILTER_SAMPLES
// #define SMOOTH_FILTER_SAMPLES   21

/*! MPU9250 olusturulan nesneler */
MPU9250_TypeDef mpu;

static double pitchSmoothArray[SMOOTH_FILTER_SAMPLES];
static double rollSmoothArray[SMOOTH_FILTER_SAMPLES];

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    /* mpu9250 sensoru var mi yok mu ? */
    if (MPU9250_WhoAmI()) {

        #ifdef SERIAL 
            Serial.println("MMPU9250 sensor is found\n");
        #endif
        /*!< Sensor kullanmaya hazir hale getirildi */
        MPU9250_init();
        /*!< Accelerometer kalibrasyon ayarlari yapildi */
        calibration_accel(&mpu);
        /*!< Gyroscope kalibrasyon ayarlari yapildi */
        calibration_gyro(&mpu);

        delay(1000);
    }
    else {    

        #ifdef SERIAL 
          Serial.println("MPU9250 sensor is not found");
        #endif

        while (1)
          ;
    }
}

void loop()
{
    /*!                                  ACCELOMETER                                       */
    /***************************************************************************************/

    /*!< Accel ham degerler alindi*/
    set_reg_accel(&mpu);
    /*!< Ham degerler alcak geciren filtreye sokuldu */
    lowPassFilter(&mpu);
    /*!< Alinan ham veriler ile acilar elde edildi */
    set_accel_deg_xyz(&mpu);

    /*!                                    GYROSCOPE                                       */
    /***************************************************************************************/

    /*!< Gyroscope ham degerler alindi */
    set_reg_gyro(&mpu);
    /*!< Alinan ham veriler ile acilar elde edildi */
    set_gyro_deg_xyz(&mpu);

    /*!                                 COMPLEMENTARY FILTER                               */
    /***************************************************************************************/

    /*!< Accelerometer'dan ve Gyroscope'dan elde edilen veriler birlestirildi */
    complementary_filter(&mpu);

    /*!< Sinyal smooth filtreye sokuldu ve uretilen deger yeni pitch degeri yapildi*/
    set_pitch(digitalSmooth(get_pitch(&mpu), pitchSmoothArray), &mpu);
    set_roll(digitalSmooth(get_roll(&mpu), rollSmoothArray), &mpu);

    #ifdef SERIAL
        Serial.print(get_accel_deg_x(&mpu));
        Serial.print(F(","));
        Serial.print(get_accel_deg_y(&mpu));
        Serial.print(F(","));
        Serial.print(get_gyro_deg_x(&mpu));
        Serial.print(F(","));
        Serial.print(get_gyro_deg_y(&mpu));
        Serial.print(F(","));
        Serial.print(get_pitch(&mpu));
        Serial.print(F(","));
        Serial.println(get_roll(&mpu));
    #endif
}
