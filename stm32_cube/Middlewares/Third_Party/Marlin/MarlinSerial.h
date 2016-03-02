/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"

#if !defined(SERIAL_PORT) 
#define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
						(port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
						(port == 3 && defined(UBRR3H)))				
						
// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (these are expanded 
// depending on selected serial port
#define M_UCSRxA SERIAL_REGNAME(UCSR,SERIAL_PORT,A) // defines M_UCSRxA to be UCSRnA where n is the serial port number
#define M_UCSRxB SERIAL_REGNAME(UCSR,SERIAL_PORT,B) 
#define M_RXENx SERIAL_REGNAME(RXEN,SERIAL_PORT,)    
#define M_TXENx SERIAL_REGNAME(TXEN,SERIAL_PORT,)    
#define M_RXCIEx SERIAL_REGNAME(RXCIE,SERIAL_PORT,)    
#define M_UDREx SERIAL_REGNAME(UDRE,SERIAL_PORT,)    
#define M_UDRx SERIAL_REGNAME(UDR,SERIAL_PORT,)  
#define M_UBRRxH SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_U2Xx SERIAL_REGNAME(U2X,SERIAL_PORT,)



#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#if MB(STM_3DPRINT)
#define BYT 0    //renamed from BYTE to BYT to avoid redefine from integer.h
#else
#define BYTE 0
#endif

#ifndef AT90USB
// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define RX_BUFFER_SIZE 128


struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};


#ifdef FLOW_CONTROL_BY_OK
// Struct flow control by OK
typedef struct {
	unsigned int nb_bytes_in_buffer;
	unsigned int nb_OK_to_send;
  unsigned int nb_OK_sent;
	bool xoff;
} flow_control_by_ok_t;

extern flow_control_by_ok_t s_flow_control;
#endif

#if UART_PRESENT(SERIAL_PORT)
  extern ring_buffer rx_buffer;
#endif
#if MB(STM_3DPRINT)
#if !defined(NO_WIFI)
extern char cmdReply[256];
extern int cmdReplyIndex;
#endif //#if !defined(NO_WIFI)
#endif
class MarlinSerial //: public Stream
{

  public:
    MarlinSerial();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);
    

#if ((defined DEBUG_RX_BUFFER)||(defined FLOW_CONTROL_BY_OK))
    FORCE_INLINE unsigned int nb_bytes_in_rx_buffer(void)
    {
    	if( gBspUartData.pRxReadBuffer <= gBspUartData.pRxWriteBuffer)
    		return (unsigned int )(gBspUartData.pRxWriteBuffer - gBspUartData.pRxReadBuffer) ;
    	else
    		return  (unsigned int )(gBspUartData.pRxWriteBuffer - gBspUartData.pRxBuffer) +
    				(unsigned int )(UART_RX_BUFFER_SIZE - (gBspUartData.pRxReadBuffer - gBspUartData.pRxBuffer));
    }
#endif
#ifdef FLOW_CONTROL_BY_OK
    FORCE_INLINE int shall_I_send_OK(void)
    {
    	unsigned int nb_bytes = nb_bytes_in_rx_buffer();


    	if( (s_flow_control.xoff == false) &&
    		(nb_bytes < 500) )
    		return 1;

    	if( (s_flow_control.xoff == true) &&
    		(nb_bytes < 300) )
    	{
    		s_flow_control.xoff = false;

    		return 1;
    	}

    	s_flow_control.xoff = true;
    	s_flow_control.nb_OK_to_send++;

    	return 0;
    }

    FORCE_INLINE unsigned int dec_nb_OK_to_send(void)
    {
    	if( (s_flow_control.xoff == false) && (s_flow_control.nb_OK_to_send>0) )
    	{
    		s_flow_control.nb_OK_to_send--;
    		return s_flow_control.nb_OK_to_send;
    	}
    	return 0;
    }

    FORCE_INLINE int get_nb_OK_sent(void)
    {
    	   return ++s_flow_control.nb_OK_sent;
   	}    
#endif   
    
    FORCE_INLINE int available(void)
    {
#if MB(STM_3DPRINT)
      return (BSP_UartGetNbRxAvalaibleBytes());
#else
      return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
#endif
    }
    
    FORCE_INLINE void write(uint8_t c)
    {
#if MB(STM_3DPRINT)
      BSP_UartIfQueueTxData(&c, 1);
#if !defined(NO_WIFI)
      buildCmdReply(c);
#endif //#if !defined(NO_WIFI)
#else
      while (!((M_UCSRxA) & (1 << M_UDREx)))
        ;

      M_UDRx = c;
#endif
    }
    
#if MB(STM_3DPRINT)
#if !defined(NO_WIFI)    
    FORCE_INLINE void buildCmdReply(char c)
    {
      cmdReply[cmdReplyIndex]=c;
      cmdReplyIndex++;
      if (cmdReplyIndex==256) cmdReplyIndex=0;
    }
#endif //#if !defined(NO_WIFI)
#endif
    
    FORCE_INLINE void checkRx(void)
    {
#if (!MB(STM_3DPRINT))  
      if((M_UCSRxA & (1<<M_RXCx)) != 0) {
        unsigned char c  =  M_UDRx;
        int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != rx_buffer.tail) {
          rx_buffer.buffer[rx_buffer.head] = c;
          rx_buffer.head = i;
        }
      }
#endif  //#if (!MB(STM_3DPRINT))  
 
    }
    
    
    private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);
    
    
  public:
    
    FORCE_INLINE void write(const char *str)
    {
      while (*str)
        write(*str++);
    }


    FORCE_INLINE void write(const uint8_t *buffer, size_t size)
    {
      while (size--)
        write(*buffer++);
    }
#if MB(STM_3DPRINT)    
    FORCE_INLINE void print(const char *str)
    {
      write(str);
    }
    
    FORCE_INLINE void printn(uint8_t *str, uint8_t nbData)
    {
      BSP_UartIfQueueTxData(str, nbData);
    }
#else
    FORCE_INLINE void print(const String &s)
    {
      for (int i = 0; i < (int)s.length(); i++) {
        write(s[i]);
      }
    }
    
    FORCE_INLINE void print(const char *str)
    {
      write(str);
    }
#endif
    
#if MB(STM_3DPRINT)   
    void print(char, int = BYT);
    void print(unsigned char, int = BYT);
#else
    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
#endif
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

#if (!MB(STM_3DPRINT))  
    void println(const String &s);
#endif
    void println(const char[]);
#if MB(STM_3DPRINT)   
    void println(char, int = BYT);
    void println(unsigned char, int = BYT);
#else
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
#endif
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
};

extern MarlinSerial MSerial;
#endif // !AT90USB

// Use the UART for BT in AT90USB configurations
#if defined(AT90USB) && defined (BTENABLED)
   extern HardwareSerial bt;
#endif

#endif
