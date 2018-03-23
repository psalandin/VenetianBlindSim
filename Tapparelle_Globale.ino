// --------------------------------------------------------------------------------
// Sketch finale del simulatore di tapparelle con Arduino per il pannello KNX
// utilizzo con attuatori JAL-0210.02 di MDT Technologies
// L'attuatore tapparelle fornisce, per ogni tapparella, una linea su ed una linea
// giù, e ciascuno di essi può essere breve o lungo. Il breve deve comandare le 
// lame delle veneziane, il lungo il motore dell'avvolgibile. 
// Si mettono insieme le animazioni tapparella e il Grab Lungo Breve
// Ora non bisogna semplicemente accendere un led per i motori, ma animare la
// tapparella, in modo che dopo un certo tempo, incontrando il fine corsa, si fermi
// comunque
// --------------------------------------------------------------------------------
// l'attuatore può essere sostituito da una coppia di pulsanti, premuti per un 
// periodo "breve" o "lungo" a seconda dell'effetto desiderato, ossia ruotare le
// lamelle o alzare/abbassare la tapparella
// --------------------------------------------------------------------------------


// --------------------------------------------------------------------------------
// i diversi tipi di led array sono configurati in modi diversi, e per questo motivo 
// il driver MD_MAX72xx deve essere impostato in modo da ottenere il corretto
// canvas da disegnare
// In questo caso, i parametri utilizzati sono stati:
// #define USE_FC16_HW 1
// mentre tutti gli altri define iniziali erano zero; inoltre i parametri
//      #define HW_DIG_ROWS
//      #define HW_REV_COLS
//      #define HW_REV_ROWS
// non sono, alla fine, sttai toccati
// Questo ha portato ad una matrice di punti in cui, orientandola con il connettore 
// in basso, lo 0,0 è in basso a sinistra, dunque la coordinata Y è corretta (sale 
// verso l'alto) mentre la X è invertita (sale verso sinistra)
// Una trasformazione lineare X1=7-X0 fa dunque tutto il lavoro
// --------------------------------------------------------------------------------
#include <MD_MAX72xx.h>
#include <SPI.h>

// --------------------------------------------------------------------------------
// Sistema di debug che utilizza la seriale; in produzione viene spento
// --------------------------------------------------------------------------------
#define  DEBUG  0
  
#if DEBUG
  #define PRINT(s, x) { Serial.print(F(s)); Serial.print(x); }
  #define PRINTS(x) Serial.print(F(x))
  #define PRINTD(x) Serial.println(x, DEC)
#else
  #define PRINT(s, x)
  #define PRINTS(x)
  #define PRINTD(x)
#endif

// pin pert i pulsanti
#define BTN_SU      2   // linea pulsante su
#define BTN_GIU     3   // linea pulsante giù
#define SU_LUNGO    4   // Uscita LED su lungo (motore)
#define GIU_LUNGO   5   // Uscita LED giù lungo (motore)
#define SU_BREVE    6   // Uscita LED su breve (lamella in apertura)
#define GIU_BREVE   7   // Uscita LED giù breve (lamella in chiusura)

#define T_SOGLIA    150 // Valore di riconoscimento della pressione lunga dalla breve
#define TEMPO_BR    200 // tempo di accensione del led giallo impulso breve (su o giù)

#define TEMPO_STEP_TAPP 500  // durata step tapparella per visualizzare che si alza o si abbassa

// pin per le tapparelle simulate con i led (MD_MAX72xx)
#define  MAX_DEVICES 4

#define CLK_PIN   13  // or SCK
#define DATA_PIN  11  // or MOSI
#define CS_PIN    10  // or SS

// --------------------------------------------------------------------------------
// stati o condizioni dei pulsanti, per il riconoscimento della pressione breve-lunga
// --------------------------------------------------------------------------------
#define NONSO       0       // non so ancora in che stato mi trovo
#define BREVE       1       // riconosciuto impulso breve, segnale alto (impulso) alle lamelle 
#define LUNGO       2       // riconosciuto impulso lungo, contatto chiuso al motore

// bisognerà anche definire i puntini che corrispondono ai led per le varie funzionalità
// ausiiarie: i fine corsa, i movimenti dei motori e le alimentazioni dai pulsanti

int btn_in[2] = {BTN_SU,BTN_GIU} ;          // array dei pin di ingresso              0=su 1=giu
int out_lungo[2] = {SU_LUNGO, GIU_LUNGO} ;  // array dei pin per i led dei motori     0=su 1=giu
int out_breve[2] = {SU_BREVE, GIU_BREVE} ;  // array dei pin per i led delle lamelle  0=su 1=giu
int val_incr[2] = {+1,-1} ;                 // incremento: su=+1 giu=-1 

//
int valore[2] = {0,0};                            // stato del pulsante rilevato dall'ingresso
//int lungo[2] = {0,0};                             // dismesso
//int breve[2] = {0,0};                             // dismesso
bool attivo[2] = {false,false};                   // suAttivo
unsigned long start[2], tempo[2], durBreve[2];    // startSu, tempoSu, durSuBreve
unsigned long startLungo[2], tempoLungo[2];       // Tempo da quando il lungo è acceso
int stato[2] = {NONSO,NONSO};                     // statoSu
int oldSt[2] = {NONSO,NONSO};                     // per rilevare il cambio di stato


// --------------------------------------------------------------------------------
// I livelli della tapparella sono in tutto 23
// I livelli di orientamento della lamella sono 5
// --------------------------------------------------------------------------------
int livTapparella, livLamella;    // Posizione della tapparella e orientamento lamelle
bool fineCorsa[2];                // valore dei fine corsa (superiore o inferiore)

int i;

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(CS_PIN, MAX_DEVICES);

// --------------------------------------------------------------------------------
// tabella posizioni delle lamelle. Corrispondono al "valore" di orientamento
// della lamella; servono per "disegnare" una linea sul Max7219; le coordinate sono 
// illustrate qui; ad esempio, la posizione intermedia iLam[2], ossia 1,1,5,5
// indica che si deve disegnare unalinea che va dal punto (1,1) al punto (5,5)
// come evidenziato
//
//    . . . . . . . .   7
//    . . . . . . . .   6
//    . . @ . . . . .   5
//    . . . @ . . . .   4
//    . . . . @ . . .   3
//    . . . . . @ . .   2
//    . . . . . . @ .   1
//    . . . . . . . .   0
//    7 6 5 4 3 2 1 0
//
// --------------------------------------------------------------------------------
int iLam[5][4] = {{0,3,6,3},
                  {0,2,6,4},
                  {1,1,5,5},
                  {2,0,4,6},
                  {3,0,3,6}
                  };

int iLedFC[2][2] = {{7,31},{7,8}};
int iLedMot[2][2] = {{7,29},{7,10}};
int iLedAl[2][2] = {{7,27},{7,12}};
                  
//#define HW_DIG_ROWS 0
//#define HW_REV_COLS 1
//#define HW_REV_ROWS 0


// ================================================================================
// questa funzione pulisce tutto (un vecchio CLS dl DOS o dei vecchi basic)
// si noti che ho scelto, per evitare sfarfallii, di eseguire a mano l'Update 
// del dispositivo ogni volta che tutto è pronto
// Ho messo un'intensitè piuttosto bassa, anche se non bassissima.
// ================================================================================
void resetMatrix(void)
{
  mx.control(MD_MAX72XX::INTENSITY, MAX_INTENSITY/8);
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  mx.clear();
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
}

// ================================================================================
// Disegno della tapparella ad un "livello" da 0 a 22
// si ricorda che, con questi parametri, il display verticale con i connettori 
// in basso ha coordinate XY che partono dall'angolo in basso a destra, Y va verso
// l'alto e X va verso sinistra
// Dovendo disegnare tapparella, lamella ed altro, il clear non appartiene alla funzione
// e in realtà nemmeno l'update!
// Disegnare delle linee vuote tra quelepiene aumenta l'impressione del movimento
// ================================================================================
void tapparella(int aiLiv) {
  int i,j;
 
  j=0;  // j è il contatore che una volta su tre disegna la linea "vuota"
  for (i=aiLiv+8;i<32;i++) {
    if (j<2) {
      mx.drawLine(0,i,5,i,true);
    } else {
      mx.drawLine(0,i,5,i,false);
    }
    j++;
    if (j>2) j=0;
  }
  // il contorno, lati e linea superiore (che ci sono sempre)
  mx.drawLine(0,aiLiv+8,0,31,true);
  mx.drawLine(5,aiLiv+8,5,31,true);
  mx.drawLine(0,31,5,31,true);
}

// ================================================================================
// orientamento di una lamella posizioni da 0 (orizzontale) a 4 (verticale)
// Dovendo disegnare tapparella, lamella ed altro, il clear non appartiene alla funzione
// e in realtà nemmeno l'update!
// Funzione banale, una volta che abbiamo già descritto le coordinate dei punti per
// ogni livello
// ================================================================================
void lamella (int aiLiv) {
  mx.drawLine(iLam[aiLiv][0],iLam[aiLiv][1],iLam[aiLiv][2],iLam[aiLiv][3],true);
}

// ================================================================================
// Disegno i led ausiliari
// migliorabile!
// Ognuno dei tre punti (tre sopra e tre sotto in realtà) viene indicato con le 
// coordinate e poi lo "stato"
// ================================================================================
void disegnaLed(int ai) {

  // fine corsa
  mx.setPoint(iLedFC[ai][0],iLedFC[ai][1],fineCorsa[ai]);
  // reale movimento motore, protetto dal fine corsa
  mx.setPoint(iLedMot[ai][0],iLedMot[ai][1],(stato[ai]==LUNGO)&&(!fineCorsa[ai]));
  // alimentazione dal pulsante o dal'attuatore
  mx.setPoint(iLedAl[ai][0],iLedAl[ai][1],(stato[ai]==LUNGO));

}

// ================================================================================
// Fattorizzo il calcolo dello stato degli ingressi
// In realtà le variabili sono globali dunque non ho una cosa veramente rientrante
// ma poco mi importa: più globali dei pin di I/O non c'è
// ================================================================================
void calcStatoIn(int ai) {
    // --------------------------------------------------------------------------------
    // Gestione del pulsante; devo decidere in che "stato" del pulsante mi trovo
    // può essere NONSO (che significa "indeciso"), BREVE o LUNGO
    // --------------------------------------------------------------------------------
    valore[ai] = digitalRead(btn_in[ai]);
    if (valore[ai]) {
      // Pulsante premuto, o attuatore che commuta in chiusura contatto
      switch (stato[ai]) {
        case NONSO:
          if (!attivo[ai]) {
            // se non ero attivo e lo sono (fronte di salita) faccio partire il timer per il riconoscimento
            // breve-lungo
            start[ai] = millis();
            #if DEBUG
            Serial.print("StartSu = ");
            Serial.println(start[ai]);
            #endif
          } else {
            // se ero già attivo devo confrontare il tempo con la soglia, per discriminare
            // il breve dal lungo; se supero il tempo di trigger, allora scatta il "lungo".
            tempo[ai] = millis();
            if (tempo[ai]>(start[ai]+T_SOGLIA)) {
              // ho superato la soglia, dunque sono LUNGO
              stato[ai] = LUNGO;
              startLungo[ai] = millis();
            }
          }
          break;
        case BREVE:
          // se sono nello stato breve e rilevo un fronte di salita significa aver premuto di nuovo
          // il pulsante nell'intervallo di 200 ms tra l'inizio e la fine del tempo breve; in realtà
          // questo non dovrebbe accadere perché l'impulso generato dall'attuatore è breve e comunque 
          // la veneziana lo sa gestire; tuttavia per il momento me ne frego e aspetto; dunque qui non
          // faccio nulla; di fatto  così il fronte non viene rilevato
          break;
        case LUNGO:
          // nello stato LUNGO significa rimanerci
          break;
      }
      attivo[ai] = true;
    }
    else {
      // --------------------------------------------------------------------------------
      // pulsante rilasciato; segnale 1 OFF
      // --------------------------------------------------------------------------------
      switch (stato[ai]) {
        case NONSO:
          if (attivo[ai]) {
            // se il tempo trascorso è poco ho impulso breve; commuto a BREVE; voglio come
            // esercizio di simulazione della veneziana accendere il led per il tempo parametrico 
            // scelto di durata del tempo breve
            tempo[ai] = millis();
            #if DEBUG
            Serial.print("TempoSu = ");
            Serial.print(tempo[ai]);
            Serial.print(" tempo-start= ");
            Serial.println(tempo[ai]-starta[i]);
            #endif
            if (tempo[ai]<(start[ai]+T_SOGLIA)) {
              stato[ai] = BREVE;
              start[ai] = millis();
            }          
          }
          break;
        case BREVE:
          // il rilascio durante il breve non ha senso perché se sono in BREVE significa che ho rilasciato
          // provo dunque a non cambiare stato
          break;
        case LUNGO:
          // Se ero nello stato LUNGO significa che il motore stava andando, e se "mollo" il contatto
          // dunque il segnale si interrompe e tornerò nello stato NONSO
          // commuto lo stato a NONSO, che di fatto ha il motore spento e le lamelle ferme
          stato[ai] = NONSO;
          break;
      }
      attivo[ai] = false;
    }
    // però se sono nello stato breve devo vedere se la durata è finita. Questo solo per il led se lo mostro
    if (stato[ai]==BREVE) {
      durBreve[ai] = millis();
      if (durBreve[ai]>start[ai]+TEMPO_BR) {
        stato[ai] = NONSO;
      }
    }
}


// ================================================================================
// controllo il cambio di stato ed agisco di conseguenza
// Qui la gestione dell'array fa un po' a pugni con il resto perché SU e GIU sono
// entità molto distinte anche se i led sono led. Vediamo cosa ne esce
// In sostanza vogliamo che un fronte di salita di un impulso breve faccia "scattare"
// un passo della lamella,e poi basta finché non trovo un altro fronte di salita
// ================================================================================
void calcCambioStato(int ai) {
  // cerco il cambio di stato da NONSO a BREVE
  if (stato[ai] != oldSt[ai]) {
    // fronte di salita breve: incremento il livello lamella; il problema è: POSSO?
    // per un primo test faccio finta che posso
    if (stato[ai]==BREVE) {
      livLamella +=val_incr[ai];
      if (livLamella<0) livLamella = 0;
      if (livLamella>4) livLamella = 4;
    }
    oldSt[ai] = stato[ai];
  }
}

// ================================================================================
// per il motore devo controllare se lo stato è LUNGO, ed in caso da quanto tempo
// lo è, perché ogni tot millisecondi di permanenza del lungo io devo alzare di
// una tacca (o abbassare!) il valore della tapparella, fino al raggiungimento
// del fine corsa, alché devo smettere di farlo
// ================================================================================
void calcTempoMotore(int ai) {
  if (stato[ai]==LUNGO) {
    tempoLungo[ai] = millis();
    if (tempoLungo[ai]>(startLungo[ai]+TEMPO_STEP_TAPP)) {
      livTapparella += val_incr[ai];
      startLungo[ai] = tempoLungo[ai];
      if (livTapparella<0) livTapparella=0;
      fineCorsa[1]= (livTapparella<=0);
      if (livTapparella>22) livTapparella=22;
      fineCorsa[0]= livTapparella>=22;
    }
  }
}

// ================================================================================
void setup() {
  // modalità dei pin di controllo pulsanti e led
  pinMode(BTN_SU,INPUT);
  pinMode(BTN_GIU,INPUT);
  pinMode(SU_LUNGO,OUTPUT);
  pinMode(GIU_LUNGO,OUTPUT);
  pinMode(SU_BREVE,OUTPUT);
  pinMode(GIU_BREVE,OUTPUT);

  // azzero stati e led
  for (i=0;i<2;i++) {
    start[i]=millis(); tempo[i]=millis(); 
    stato[i] = NONSO;
  }
  // spengo tutto
  digitalWrite(SU_LUNGO,false);
  digitalWrite(SU_BREVE,false);
  digitalWrite(GIU_LUNGO,false);
  digitalWrite(GIU_BREVE,false);
  
  // inizializzazione matrrice
  mx.begin();
  resetMatrix();
  #if DEBUG
    Serial.begin(57600);
  #endif
  PRINTS("\n[MD_MAX72XX Pacman]");

  // stato logico del sistema
  livTapparella = 22;           // tapparella tutta su
  livLamella = 0;               // lamella tutta aperta, ovviamente (orizzontale)
  fineCorsa[0] = true;          // sono in alto, il fine corsa in alto è attivo
  fineCorsa[1] = false;         // e quello in basso ovviamente no
  
}

// ================================================================================
void loop() {

  for (i=0;i<2;i++) {
    // Calcolo lo stato degli ingressi
    calcStatoIn(i);
    // calcolo gli eventuali fronti di salita (per chi lavora a fronte)
    calcCambioStato(i);
    // proviamo il tempo motore
    calcTempoMotore(i);
    // mostro lo stato degli ingressi (per ora con i 4 led)
    switch (stato[i]) {
      case NONSO:
        digitalWrite(out_lungo[i],false);
        digitalWrite(out_breve[i],false);
        break;
      case BREVE:
        digitalWrite(out_lungo[i],false);
        digitalWrite(out_breve[i],true);
        break;
      case LUNGO:
        digitalWrite(out_lungo[i],true);
        digitalWrite(out_breve[i],false);
        break;
    }
  }
  // ora mostro le tapparelle
  mx.clear();
  tapparella (livTapparella);
  lamella (livLamella);
  for (i=0;i<2;i++) {
    disegnaLed(i);
  }
  mx.update();

}
