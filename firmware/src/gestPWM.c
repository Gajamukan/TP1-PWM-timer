/*--------------------------------------------------------*/
// GestPWM.c
/*--------------------------------------------------------*/
//	Description :	Gestion des PWM 
//			        pour TP1 2023-2024
//
//	Auteur 		: 	G. Subramaniyam
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V5.50 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include "system_config.h"
#include "system_definitions.h"
#include "peripheral/oc/plib_oc.h"

#include "app.h"
#include "GestPWM.h"
#include "Mc32DriverLcd.h"
#include "Mc32DriverAdc.h"
#include "bsp.h"

APP_DATA appData;


void GPWM_Initialize(S_pwmSettings *pData)
{
   // Init les data
    pData->AngleSetting = 0;
    pData->SpeedSetting = 0;
    pData->absAngle = 0;
    pData->absSpeed = 0;
    
   // Init état du pont en H
    STBY_HBRIDGE_W = 0;   // STBY Low
    
   // lance les timers et OC
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();

    DRV_OC0_Start();
    DRV_OC1_Start(); 
}

// Obtention vitesse et angle (mise a jour des 4 champs de la structure)
void GPWM_GetSettings(S_pwmSettings *pData)	
{
    //Structure des 2 canaux ADC
    S_ADCResults AdcResults;
    
    //Tableaux pour les 10 dernières valeurs des deux canaux ADC
    uint16_t TabADC1[10];
    uint16_t TabADC2[10];
    
    // Variables pour calculer la moyenne de la vitesse et de l'angle
    uint16_t MoyenneVitesse, MoyenneAngle;
    
    // Variables pour stocker la somme des valeurs
    uint32_t SommeVitesse = 0;
    uint32_t SommeAngle = 0;
    
    // Indices de boucle
    int i,j ;
    
    // Lecture du convertisseur AD
    AdcResults = BSP_ReadAllADC();    
    
    // Remplissage des tableaux avec les 10 dernières valeurs des canaux ADC
    for (i = 0; i < 10; i++ )
    {
       TabADC1[i] = AdcResults.Chan0;
       TabADC2[i] = AdcResults.Chan1 ;
    }
    
    // Calcul de la somme des valeurs
    for (j = 0; j < 10; j++ )
    {
       SommeVitesse += TabADC1[j] ;
       SommeAngle += TabADC2[j] ;
    }
    
    // Calcul des moyennes
    MoyenneVitesse = SommeVitesse / 10;
    MoyenneAngle = SommeAngle / 10;

    // Conversion vitesse et mise à jour de la structure
    pData->SpeedSetting = ( ( MoyenneVitesse * 198) / 1023 ) - 99;
    
    // Conversion angle et mise à jour de la structure
    pData->AngleSetting = ( ( MoyenneAngle * 180) / 1023 ) - 90;
    
    //Vitesse en absolue
    pData->absSpeed = (abs(pData->SpeedSetting + 99))/2;
    
    //Angle en absolue
    pData->absAngle = (abs(pData->AngleSetting + 90)); 
}

// Affichage des information en exploitant la structure
void GPWM_DispSettings(S_pwmSettings *pData)
{
    //Ecriture sur la deuxième ligne
    lcd_gotoxy(1,2);
    printf_lcd("SpeedSetting %3d %%", pData->SpeedSetting );

    //Ecriture sur la troisième ligne
    lcd_gotoxy(1,3);
    printf_lcd("AbsSpeed %2d %%", pData->absSpeed);

    //Ecriture sur la quatrième ligne
    lcd_gotoxy(1,4);
    printf_lcd("Angle %3d°", pData->AngleSetting);
}

// Execution PWM et gestion moteur à partir des info dans structure
void GPWM_ExecPWM(S_pwmSettings *pData)
{
    int16_t PulseWidthOC2, PulseWidthOC3;

    // Sens horaire
    if(pData->SpeedSetting < 0)
    {
        AIN1_HBRIDGE_W = 1; 
        AIN2_HBRIDGE_W = 0; 
        STBY_HBRIDGE_W = 1; 
    }
    // Sens anti horaire
    if (pData->SpeedSetting > 0)
    {
        AIN1_HBRIDGE_W = 0; 
        AIN2_HBRIDGE_W = 1; 
        STBY_HBRIDGE_W = 1; 
    }
    // Standby
    if (pData->SpeedSetting == 0)
    {
        STBY_HBRIDGE_W = 0;      
    }

    PulseWidthOC2 = (1999 * pData->absSpeed) / 100;
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, PulseWidthOC2);
    
    PulseWidthOC3 = 2999 + ((8999 * pData->absAngle)/ 180);
    PLIB_OC_PulseWidth16BitSet(OC_ID_3, PulseWidthOC3);

}

// Execution PWM software
void GPWM_ExecPWMSoft(S_pwmSettings *pData)
{
    static uint8_t PWM_Cnt = 0;
    
    PWM_Cnt ++;
     
    if (PWM_Cnt <= pData->absSpeed)
    {
        BSP_LEDOff(BSP_LED_2);
    }
    else if (PWM_Cnt > 100)
    {
        PWM_Cnt = 0;
    }
    else
    {
        BSP_LEDOn(BSP_LED_2);
    }
}


