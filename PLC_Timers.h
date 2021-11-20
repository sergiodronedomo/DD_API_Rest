

struct timer_def
{
unsigned long PV;     //Present value. salida analogica del timer
unsigned long PSET;   //ENTRADA: Es el preset  del timer en  decimas de Seg   0.1 segs = 100 mSeg
 bool EN;         //ENTRADA: habilita el timer
 bool OUT;        //salida digital del timer
 bool PREV;
 bool POS;
 bool AST;        // ENTRADA: lo configura como astable   se resetea solo al terminar y oscila
};

typedef struct timer_def PLC_Timer;
