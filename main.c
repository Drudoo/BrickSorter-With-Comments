//
//  main.c
//  ColorSorter
//
//  PIC-1
//
//  Created by Frederik Vanggaard on 4/2/11.
//  Copyright 2011 Drudoo Entertainment. All rights reserved.
//

/*
*********************************************************************
*                                                                   *
*    Meget af denne kode er ens med den forrige,                    *
*    der er derfor ikke kommenteret pŒ de ting som er               *
*    identisk med den forrige kode.                                 *
*                                                                   *
*********************************************************************
*/



#pragma config =0x3fdf //MCLRE =off
#pragma config &=0x3ff7 //wdt =off
#pragma config &=0x3ffc //intrc-osc-clkout off

#include "int16CXX.h"

//Funktioner 
//For at programmet kan k¿re skal vi have nogle funktioner som burges til at afvikle programmet.
void timerDelay() ;
void longDelay();
void convertValAD();
void getColorInput();
void sendInputToSecondPic();
void getQuestion();

uns16 valAD;
uns8 color, needData, data;

void main()
{
	TRISC=0;
	TRISA=0;
	TRISB=0;
	PORTB=0;
	PORTA=0;
	PORTC=0;
    
    TRISB.7=1;
    
    TX9=0; 
    SPEN=1;
    TRMT=0;
    TX9D=0;
    SYNC=0;
    BRGH=1;
    BRG16=1;
    SPBRG= 103;
    TXEN=1;
    

	TRISA.0=1;//AN0 INPUT
	ANSEL.0=1; //AN0 analog
	ADCON0=0b10000000; //h¿jre justeret og AN2s
	ADCON1=0x00;
	
    
    //Vi starter vores while loop og den indeholder vores program.
	while (1) {	
        
        //Vores program er ret kort. Det meste ligger ikke i vores main funktion.
        //Vores f¿rste funktion er getQuestion. Den bliver kaldt hele men funktionen begynder ikke f¿r den for besked fra PIC-2. 
        //Det kan ses i dens funktion senere i koden.
		getQuestion();
        
        //Vi har et meget kort delay.
        timerDelay(160);

    }
}


//Vi sender og modtager data pr¾cis som i programmet fra PIC-2. 
uns8 modtagData() {
	uns8 data;
	data = 0;

    CREN = 0;
	CREN = 1;
    
	while(!RCIF) {
    }
	
    data = RCREG;
	RCIF = 0;
    return data;
}

void sendData(uns8 ud) {
    
	while(!TXIF) {
    }
	
    TXREG=ud;
	TXIF=0;
}


//Vores funktion, som bliver kaldt i while(1) loop'et. BestŒr at mange forskellige funktioner.
void getQuestion() {
	
    //F¿rst s¾tter vi v¾rdien needData til at v¾re lig med vores modtagData. Fordi PIC-2 sender tallet 1 til PIC-1 vil needData v¾re lig med 1.
    needData = modtagData();

    //Vi siger sŒ at needData skal v¾re 1 f¿r resten af funktionen kan bruges. Hvis vi ikke har fŒet et signal fra PIC-2 sŒ vil needData ikke v¾re 1 og derfor skal funktionen ikke g¿re noget.
	if(needData) {
        
        //Vi skal nu konvertere vores v¾rdi fra fotoresistoren til en v¾rdi som kan l¾ses af PIC'en.
		convertValAD();
        
        //Derefter skal vi definere hvad color er med et tal fra 1-6. 
        getColorInput();
        
        //Til sidst skal vi sende det tal til PIC-2
        sendInputToSecondPic();
	}
}



void convertValAD() {

    ADCON0.0=1; //ENABLE ADC 
    timerDelay(80);
    ADCON0.1=1; //START ADC
    
    //vent til konvertering er afsluttet
    
    while(!ADCON0.1)
        nop();
    
        //Vi gemmer vores v¾rdi i variablen valAD.
    valAD = ADRESH*256+ADRESL;
}


//Til at starte med var det meningen at vores robot skulle sortere 6 farver. Men vi havde et par problemer pŒ vejen og endte med at have 3 farver.
//Koden kan godt klare 3 farver vi har bare ikke haft tid til at teste og mŒle pŒ de sidste 3.

void getColorInput() {
            
        //  color
        //  red = 1
        //  green = 2
        //  yellow = 3
        //  black = 4
        //  white = 5
        //  blue = 6
            
        //Udregningen fra volt til valAD er sp¾nding*1024/5
    
    if((valAD > 369)&&(valAD <= 471)) {
        color = 1  ;  //red 1.8 - 2.3
    }

    if((valAD > 0)&&(valAD <= 368)) {
        color = 2;  //green 0-1.8
    }
    
    /*
    if((valAD > 327)&&(valAD <= 389)) {
        color = 3;  //yellow 
    }
    
    if((valAD > 122)&&(valAD <= 163)) {
        color = 4;  //black 
    }
    */
    
    if((valAD > 472)&&(valAD <= 1023)) {
        color = 5;  //white 2.3-5
    }
    

    /*
    if((valAD > 348)&&(valAD <= 399)) {
        color = 6;
    }
    */          
            
}


//NŒr vi har fundet ud af hvad farven er og givet den et nummer sŒ sender vi nummeret til PIC-2
void sendInputToSecondPic() {
    
	sendData(color);
    
}




//Giver n millesekund delay
void timerDelay(uns16 n) {
    
    INTCON.2=0;
	OPTION_REG=2;
	TMR0 = -125	;
	
    do {
		
        do {
        }
        
		while(!INTCON.2);
		TMR0=-125;
		INTCON.2=0;
	}
	
    while(--n>0);
    
}