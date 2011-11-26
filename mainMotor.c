//
//  main.c
//  ColorSorter
//
//  PIC-2
//
//  Created by Frederik Vanggaard on 4/2/11.
//  Copyright 2011 Drudoo Entertainment. All rights reserved.
//



#pragma config =0x3fdf //MCLRE =off
#pragma config &=0x3ff7 //wdt =off
#pragma config &=0x3ffc //intrc-osc-clkout off

#include "int16CXX.h"


//Variabler 
//Vi definere de variabler som vi skal bruge i vores program.
bit t0, t1, t2, t3, t4, t5; //Vi bruger nogle bit variabler til at check hvorlangt i vores program vi er. I vores main loop kan hver funktion ikke k�re f�r den forrige funktion er f�rdig.


//Funktioner 
//For at programmet kan k�re skal vi have nogle funktioner som burges til at afvikle programmet.

void convertValAD();
void sendBrickToSensor();
void getSensorInput();
void whereToPutBrick();
void deliverBrick();
void resetDirections();
void timerDelay(uns16);
void askForData();


uns16 valAD;
uns8 data, color, gldata;


void main() {       //Her bliver selve programmet afviklet. 


    //Vi starter med at nulstille alle vores porte.

	TRISA=0;    //Vi angiver f�rst vores udgangsporte.
	TRISB=0;
	TRISC=0;
    
	PORTA=0;    //Derefter angiver vi vore indgangsporte.
	PORTB=0;
	PORTC=0;

    //I vores robot bruger vi 2 PICKit 16F690. For at de kan snakke sammen skal vi have en serial kommunikation mellem dem. 
    //Vi bruger en to-vejs kommunikation og derfor skal vi s�tte porgrammet op med b�de modtager og sender. 
    //Meget af det som modtageren skal have sat op bruger senderen ogs�. Vi kan derfor n�jes med at skrive det meste en gang.
    
    //F�rst initialisere vi BRGH og BRG16 s� vi kan f� den baud rate vi har brug for fra SPBRG.
    BRGH=1;
	BRG16=1;
    
    //De to PICKit skal bruge den samme baud rate og vi v�lger 9600. For at bruge det skal vi s�tte SPBRG til 103.
    SPBRG=103;
    
    //Derefter enabler vi serial porten SPEN.
    SPEN=1;
    
    //Da vi ikke bruger 9bit, men i stedet 8bit s� enabler vi ikke RX, TX og TX9D. 
	RX9=0;      
    TX9=0;
    TX9D=0;

    //Vi enabler nu CREN s� vi kan modtage et signal.
    CREN=1;
    
    //For at f� en asynkron drift skal SYNC v�re clear. 
    SYNC=0;

    //For at enable senderen skal vi s�tte TXEN op. N�r TXEN bliver sat, s� blver TXIF ogs� sat automatisk. Vi vil senere bruge TXIF.
    TXEN=1;
    
    //TRMT indikere om der senderen er idle eller aktiv.
	TRMT=0;
    
    //Vores test bits bliver alle sammen sat til 0 for at programmet kan starte.
	t0 = 0;
	t1 = 0;
	t2 = 0;
	t3 = 0;
	t4 = 0;
	t5 = 0;

	
    
    //Vi er nu f�rdige med vores ops�tning og kan g� igang med vores funktioner.
    
    //Vores f�rste funktion ligger uden for vores while(1) lykke. Det g�r at den kun bliver kaldt f�rste gang programmet k�rer. 
    //Vores robotarm kan v�re i hvilken som helst possition n�r den er slukket og n�r programmet startes s� skal den k�re til startpossitionen. Det sker n�r vi kalder resetDirections.
	resetDirections();

    
    //Vi starter nu vores while loop og den indeholder resten af vores program.
	while (1) {	



        //For at programmet kan begynde skal t0-t5 v�re 0. Det g�r vi for at sikre at alle vores main() er blevet udf�rt korrekt. 
        //Hvis ikke at t0-5 er 0 s� kan vores funktion ikke starte og resten af programmet kan derfor heller ikke k�re. 
		if(!t0&&!t1&&!t2&&!t3&&!t4&&!t5) {
            
            //Vi kalder nu vores f�rste funktion inde i while(1) loop'et. Det er en funktion som skal sende vores klods fra slisken til vores sensor og robotarm. 
            sendBrickToSensor(); 		
            
            //N�r den funktion er udf�rt s� s�tter vi t0 til at v�re 1. 
            t0 = 1;
		}
		
		//For at give robotten lidt tid, s� det hele ikke sker alt for hurtigt s� har vi et delay p� 1000 milisekunder eller 1 sekund. 
        timerDelay(1000);
			
        
        //For at k�re den n�ste funktion skal t0 er lig med 1. Hvis den forrige funktion ikke er blevet k�rt korrekt s� vil t0 v�re lig med 0 og derfor kan denne funktion ikke k�re. 
		if(t0) {
            
            
            /*
            Vi havde mange problemer med at f� den rigtige farve. 
            Vores problem var at PIC'en med styrede sensoren, PIC-1, hele tiden sendte farve signaler til vores anden PIC, som styrede motoren, PIC-2. 
            Det var derfor ikke altid den rigtige farve som PIC-2 modtog fra PIC-1. 
             
            Problemet kan illustreres ved at sige at en mand st�r og banker p� d�ren hele tiden. En gang i mellem �bner vi d�ren og h�re p� hvad han har at sige. 
            Nogle gang er det bare ikke det helt ny information han fort�ller os. 
            Vi rettede op p� problemet ved at manden ikke banker p� d�ren hele tiden for at give os information, 
            men bare st�r og venter p� at vi �bner d�re og sp�rg efter den nyeste information.
            */
            
            
            //Vi beder nu vores anden PIC om at f� et farve input fra sensoren.
            askForData();
            
            //N�r vi har bedt om et farve input, s� s�tter vi t0 til 0 og t1 til 1. Nu ved vores program at askForData er blevet udf�rt korrekt og at vi kan g� vidre til n�ste funktion. 
            t0 = 0;
            t1 = 1;
		}

        //Som f�r s� bliver der checket om alle tidligere funktioner er udf�rt korrekt f�r denne funktionen begynder.
		if(t1) {
            
            //I forrige funktion bedte vi PIC-1 og en farve input og det er her at PIC-2 nu modtager dette input. 
            getSensorInput();
            
            t1 = 0;
            t2 = 1;
		}


 		if(t2) {
            
            //N�r vi har f�et et input fra sensoren s� skal vi finde ud af hvilken farve det er en hvor den skal l�gges.
            //Denne funktion bestemmer p� baggrund af getSensorInput, hvor langt vores robotarm skal dreje f�r den smider klodsen af.
            whereToPutBrick();
            
            t2 = 0;
            t3 = 1;
		}
		
		if(t3) {
            
            //N�r vores robotarm har drejet der hend hvor den skal v�re s� skal klodsen skubbes ud af armen og ned i en beholder. 
            //Denne funktion s�rger for dette og som i de andre funktioner s� kan den ikke k�res f�r de andre er f�rdige. 
            deliverBrick();
            t3 = 0;
            t4 = 1;
		}
		
		if(t4) {
            
            //N�r vi har leveret vores klods, s� er det p� tide at vende tilbage til start possitionen og begynde forfra.
            //Her bruger vi den samme funktion som vi havde i starten.
            resetDirections();
            t4 = 0;
		}
		
        //Vi er nu f�rdige med vores while(1) loop og alle vores t0-t5 v�rdier er igen 0. Det loop'et kan nu k�res igen uden af f� nogle errors p� grund af tidligere gennemk�rsler. 
		
    }
}


/*
    Indtil vidre har vi ikke skrevet hvad vores funktioner skal g�re. Dette er det gode ved test-driven development. 
    Alle burde kunne forst� hvad der er skrevet i vores while loop og ved hvad programmet skal g�re. 
    Vi skal nu til at skrive alle vores funktioner og hvad de g�r. 
 
    Funktionerne her bliver ikke kaldt i den r�kkef�lge de er i her. De bliver kaldt som de st�r i void main funktionen. 
    Det er derfor lige meget hvilken r�kkef�lge de st�r i her, men for at g�re det lettere for os selv og fordi vi bruger test-driven development s� st�r de n�sten altid i rigtig r�kkef�lge.

    Vores f�rste funktion skal resette vores robotarm. Det g�r den ved f�rst at checke om vores kontakt er trykket ind eller ej. 
    Kontakten virker p� den m�de at hvis den er trykket ind s� l�ber der ikke nogen sp�nding i gennem den. Hvis den ikke er trykket ind s� l�ber der 5 volt i gennem den.
    I denne funktion s� checker vi om den er ikke er trykket ind. 
 */
void resetDirections () {

        //Hvis kontakten ikke er trykket ind s� skal vores motor k�re i en uendelighed. 
        //N�r kontakten s� bliver trykket ind, s� til while loop'et stoppe og programmet vil g� videre til n�ste del af funktionen. 
	while(PORTA.4) {    //Vores PORTA.4 er ben 3 p� PIC'en. 

		PORTC.0 = 1;   
        PORTC.6 = 0;    

/*
    PORTC.0 og PORTC.6 er hendholdsvis ben 16 og 8 p� PIC'en. De g�r fra PIC'en hend til vores H-bro som st�r for motorstyringen og derefter videre til selve motoren. 
    N�r PORTC.6 er 0 betyder det at der ikke l�ber en sp�nding hend til H-broen. N�r PORTC.0 tilg�ng�ld er 1 s� l�ber der en sp�nding p� 5 volt til H-broen,
    som s� sender 9volt hend til motoren.
*/
        
    }
        
    //N�r kontakten s� bliver trykket ind stopper vores motor med at k�re og funktionen er f�rdig. 
	PORTC.0 = 0;
	PORTC.6 = 0;


}


/*
    Det n�ste vi skal have gjort er at sende vores klods til sensoren. Det er egentlig gjort p� samme m�de som i forrige funktion. 
    Vi starter med at �bne for PORTC.1 (ben 15) og holder PORTC.2 (ben 14) lukket.
    Derefter venter vi i 450 millisekunder eller 0.45 sekunder og s� g�r vi det omvendte for at f� vores motor til at k�re den anden vej. 
    Vi har igen et delay, og til slut lukker vi for begge porte s� motoren ikke k�re nogen steder. 
    For at k�re hele processen lidt langsommere tilf�jer vi en pause p� 1000 millisekunder (1 sekund)
*/

void sendBrickToSensor()  {
    

		PORTC.1 = 1;
		PORTC.2 = 0;

		timerDelay(450);

		PORTC.1 = 0;
		PORTC.2 = 1;

		timerDelay(450);

		PORTC.1 = 0;
		PORTC.2 = 0;

		timerDelay(1000);

}


/*
    Vi skal nu til at sende og modtage signaler. Den serielle kommunikation mellem PIC-1 og PIC-2 har v�ret vores st�rste problem i programmeringen. 
    Vi starter med at have en 8bit variable som hedder modtagData. Den modtager data fra PIC-1 og gemmer det, s� vi kan kalde 'modtagData();' senere og bruge det i vores funktion. 
*/

uns8 modtagData() {
	uns8 data;
	data = 0;

/*    
    I starten af vores 'void main' satte vi CREN = 1. Det gjorde vi fordi hvis den er 0, s� kan vores PIC ikke modtage et signal. 
    Vi fik ret store problemer fordi efter 2 omgange af vores while(1) loop s� stoppede vi med at f� et signal fra PIC-1.
    Vi fandt efter lang tid ud af at der i datasheetet for PIC'en st�r: (side 159, afsnit 12.1.2.5)
 
 
        *********************************************************************
        *                                                                   *
        *                Receive Overrun Error                              *
        *    The receive FIFO buffer can hold two characters. An            *
        *    overrun error will be generated If a third character, in its   *
        *    entirety, is received before the FIFO is accessed. When        *
        *    this happens the OERR bit of the RCSTA register is set.        *
        *    The characters already in the FIFO buffer can be read          *
        *    but no additional characters will be received until the        *
        *    error is cleared. The error must be cleared by either          *
        *    clearing the CREN bit of the RCSTA register or by              *
        *    resetting the EUSART by clearing the SPEN bit of the           *
        *    RCSTA register.                                                *
        *                                                                   *
        *********************************************************************
 
 Det bet�d at fordi vores CREN altid var 1, og derfor aldrig blev cleared, s� blev der lavet en overrun error n�r den fix mere end 3 tal. 
 Det vi derfor gjorde var at s�tte CREN = 0 for at sikre os at den var cleared og derefter s�tte CREN = 1 igen, s� vi igen kan modtage et signal.
 Vi modtager data p� ben 12 (RB5/RX) og sender p� ben 10 (RB7/TX)
 */
 
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

//Vi har nu vores funktion, hvor vi sp�rger PIC-1 og et input fra sensoren. Vi sender derfor et signal fra PIC-2 til PIC-1. Signalet er meget simpelt og indeholder kun et ettal (1).
void askForData() {
	sendData(1);
	
}


//N�r PIC-1 har modtaget vores signal s� sender den en farve tilbage. Det g�r den i form at tal fra 1-6. 
//Vi modtager tallet som modtagData();, men for at g�re det lettere for os selv og programmeringen s� skriver vi at color skal v�re lig med modtagData();. 
//Det g�r at vi fremover bare kan kalde color og s� f�r vi vores modtagData();
void getSensorInput() {
  
	color = modtagData();	
    
}


/*
    N�r vi har f�et farven s� skal vi finde ou af hvor vi skal ligge vores klods. Det g�r vi ved at sige at hvis color er lig med 1, 
    s� skal vi dreje robotarmen i 500 millisekunder (0.5 sekunder) og derefter stoppe. 
    I tidligere funktioner har vi bagefter at have k�rt motoren den ene vej, s� k�rt den anden vej. 
    Det skal vi ikke her fordi vi i mellemtiden skal stubbe klodsen ud af vores arm med en anden funktion.
*/

void whereToPutBrick() {
  

	if (color == 1) {

		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(500);

		PORTC.6 = 0;
	} 



	if (color == 2) {


		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(1000);

		PORTC.6 = 0;


	}
	


	if (color == 3) {
		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(1500);

		PORTC.6 = 0;
	}	



	if (color == 4) {

		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(2300);

		PORTC.6 = 0;
	}
	


	if (color == 5) {
		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(2800);

		PORTC.6 = 0;
	}
	

	if (color == 6) {

		PORTC.0 = 0;
		PORTC.6 = 1;

		timerDelay(2700);

		PORTC.6 = 0;

	}

	


}

//Her skubber vi klodsen ud af vores robotarm og denne funktion g�r det samme som sendBrickToSensor.
//De ben vi bruger her er PORTC.3 (ben 7) og PORTC.4 (ben 6)

void deliverBrick() {

		PORTC.3 = 0;	
		PORTC.4 = 1;	

		timerDelay(150);

		PORTC.3 = 1;
		PORTC.4 = 0;
		
		timerDelay(150);
		
		PORTC.3 = 0;

		timerDelay(1000);
}


//Den sidste funktion er vores timer. PIC'en har en indbygger timer funktion og det er den vi har brugt til alle vores delays. 

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