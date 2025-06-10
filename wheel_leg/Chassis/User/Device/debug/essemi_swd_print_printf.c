/**
  *********************************************************************************
  *
  * @file    essemi_swd_print_printf.c
  * @brief   Replacement for printf to write formatted data via swd-print.
  *
  * @version V1.0
  * @date    2021.03
  * @author  AE Team
  * @note
  *
  * Copyright (C) Shanghai Eastsoft Microelectronics Co. Ltd. All rights reserved.
  ******************************************************************************
  */

#include "essemi_swd_print.h"
#include "essemi_swd_print_conf.h"

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

#ifndef essemi_swd_printf_BUFFER_SIZE
#define essemi_swd_printf_BUFFER_SIZE (64)
#endif

#include <stdlib.h>
#include <stdarg.h>


#define FORMAT_FLAG_LEFT_JUSTIFY   (1u << 0)
#define FORMAT_FLAG_PAD_ZERO       (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN     (1u << 2)
#define FORMAT_FLAG_ALTERNATE      (1u << 3)

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

typedef struct
{
    char     *pBuffer;
    unsigned  BufferSize;
    unsigned  Cnt;

    int   ReturnValue;

    unsigned SWDBufferIndex;
} essemi_swd_printf_DESC;

/*********************************************************************
*
*       Function prototypes
*
**********************************************************************
*/

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _StoreChar
*/
static void _StoreChar(essemi_swd_printf_DESC *p, char c)
{
    unsigned Cnt;

    Cnt = p->Cnt;

    if ((Cnt + 1u) <= p->BufferSize)
    {
        *(p->pBuffer + Cnt) = c;
        p->Cnt = Cnt + 1u;
        p->ReturnValue++;
    }

    //
    // Write part of string, when the buffer is full
    //
    if (p->Cnt == p->BufferSize)
    {
        if (ESSEMI_SWD_Write(p->SWDBufferIndex, p->pBuffer, p->Cnt) != p->Cnt)
        {
            p->ReturnValue = -1;
        }
        else
        {
            p->Cnt = 0u;
        }
    }
}

/*********************************************************************
*
*       _PrintUnsigned
*/
static void _PrintUnsigned(essemi_swd_printf_DESC *pBufferDesc, unsigned v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags)
{
    static const char _aV2C[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    unsigned Div;
    unsigned Digit;
    unsigned Number;
    unsigned Width;
    char c;

    Number = v;
    Digit = 1u;
    //
    // Get actual field width
    //
    Width = 1u;

    while (Number >= Base)
    {
        Number = (Number / Base);
        Width++;
    }

    if (NumDigits > Width)
    {
        Width = NumDigits;
    }

    //
    // Print leading chars if necessary
    //
    if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u)
    {
        if (FieldWidth != 0u)
        {
            if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (NumDigits == 0u))
            {
                c = '0';
            }
            else
            {
                c = ' ';
            }

            while ((FieldWidth != 0u) && (Width < FieldWidth))
            {
                FieldWidth--;
                _StoreChar(pBufferDesc, c);

                if (pBufferDesc->ReturnValue < 0)
                {
                    break;
                }
            }
        }
    }

    if (pBufferDesc->ReturnValue >= 0)
    {
        //
        // Compute Digit.
        // Loop until Digit has the value of the highest digit required.
        // Example: If the output is 345 (Base 10), loop 2 times until Digit is 100.
        //
        while (1)
        {
            if (NumDigits > 1u)         // User specified a min number of digits to print? => Make sure we loop at least that often, before checking anything else (> 1 check avoids problems with NumDigits being signed / unsigned)
            {
                NumDigits--;
            }
            else
            {
                Div = v / Digit;

                if (Div < Base)          // Is our divider big enough to extract the highest digit from value? => Done
                {
                    break;
                }
            }

            Digit *= Base;
        }

        //
        // Output digits
        //
        do
        {
            Div = v / Digit;
            v -= Div * Digit;
            _StoreChar(pBufferDesc, _aV2C[Div]);

            if (pBufferDesc->ReturnValue < 0)
            {
                break;
            }

            Digit /= Base;
        }
        while (Digit);

        //
        // Print trailing spaces if necessary
        //
        if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == FORMAT_FLAG_LEFT_JUSTIFY)
        {
            if (FieldWidth != 0u)
            {
                while ((FieldWidth != 0u) && (Width < FieldWidth))
                {
                    FieldWidth--;
                    _StoreChar(pBufferDesc, ' ');

                    if (pBufferDesc->ReturnValue < 0)
                    {
                        break;
                    }
                }
            }
        }
    }
}

/*********************************************************************
*
*       _PrintInt
*/
static void _PrintInt(essemi_swd_printf_DESC *pBufferDesc, int v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags)
{
    unsigned Width;
    int Number;

    Number = (v < 0) ? -v : v;

    //
    // Get actual field width
    //
    Width = 1u;

    while (Number >= (int)Base)
    {
        Number = (Number / (int)Base);
        Width++;
    }

    if (NumDigits > Width)
    {
        Width = NumDigits;
    }

    if ((FieldWidth > 0u) && ((v < 0) || ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)))
    {
        FieldWidth--;
    }

    //
    // Print leading spaces if necessary
    //
    if ((((FormatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) || (NumDigits != 0u)) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u))
    {
        if (FieldWidth != 0u)
        {
            while ((FieldWidth != 0u) && (Width < FieldWidth))
            {
                FieldWidth--;
                _StoreChar(pBufferDesc, ' ');

                if (pBufferDesc->ReturnValue < 0)
                {
                    break;
                }
            }
        }
    }

    //
    // Print sign if necessary
    //
    if (pBufferDesc->ReturnValue >= 0)
    {
        if (v < 0)
        {
            v = -v;
            _StoreChar(pBufferDesc, '-');
        }
        else if ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)
        {
            _StoreChar(pBufferDesc, '+');
        }
        else
        {

        }

        if (pBufferDesc->ReturnValue >= 0)
        {
            //
            // Print leading zeros if necessary
            //
            if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) && (NumDigits == 0u))
            {
                if (FieldWidth != 0u)
                {
                    while ((FieldWidth != 0u) && (Width < FieldWidth))
                    {
                        FieldWidth--;
                        _StoreChar(pBufferDesc, '0');

                        if (pBufferDesc->ReturnValue < 0)
                        {
                            break;
                        }
                    }
                }
            }

            if (pBufferDesc->ReturnValue >= 0)
            {
                //
                // Print number without sign
                //
                _PrintUnsigned(pBufferDesc, (unsigned)v, Base, NumDigits, FieldWidth, FormatFlags);
            }
        }
    }
}

/*********************************************************************
*
*       _PrintFloat
*/
static void _PrintFloat(essemi_swd_printf_DESC *pBufferDesc, double v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags)
{
    unsigned  Width;
    double    Number;
    int       mul;
    int       len;
    int       int_part;
    int       int_part_len;
    // int        decimal_part_len;
    int       decimal_part;
    const char _aV2C[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    Number = (v < 0) ? -v : v;

    /*
    * Get actual field width
    */
    Width = 2u;
    int_part_len = 1u;

    /* Get length of integer part */
    while (Number >= (int)Base)
    {
        Number = (Number / (int)Base);
        Width++;
        int_part_len++;
    }

    Width += NumDigits;

    /*
    * Calculate the field length
    */
    /* Judge whether the totoal field length need to minus a symbol bit length of sign */
    if ((FieldWidth > 0u) && ((v < 0) || ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)))
    {
        FieldWidth--;
    }

    /* Get the length of total field */
    if (FieldWidth < Width)
    {
        FieldWidth = Width;
    }

    /* minus a symbol bit length of '.' */
    FieldWidth--;

    /*
    * Print leading spaces if necessary
    */
    if (pBufferDesc->ReturnValue >= 0)
    {
        if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u))
        {
            if (FieldWidth - NumDigits != 0u)
            {
                while ((FieldWidth != 0u) && (int_part_len < FieldWidth - NumDigits))
                {
                    FieldWidth--;
                    _StoreChar(pBufferDesc, ' ');
                }
            }
        }
    }

    /*
    * Print sign if necessary
    */
    if (pBufferDesc->ReturnValue >= 0)
    {
        if (v < 0)
        {
            v = -v;
            _StoreChar(pBufferDesc, '-');
        }
        else if ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)
        {
            _StoreChar(pBufferDesc, '+');
        }
        else
        {
            ;
        }

        if (pBufferDesc->ReturnValue >= 0)
        {
            /*
            * Print leading zeros if necessary
            */
            if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (1))
            {
                if (FieldWidth != 0u)
                {
                    while ((FieldWidth != 0u) && (int_part_len < FieldWidth - NumDigits))
                    {
                        FieldWidth--;
                        _StoreChar(pBufferDesc, '0');

                        if (pBufferDesc->ReturnValue < 0)
                        {
                            break;
                        }
                    }
                }
            }

            if (pBufferDesc->ReturnValue >= 0)
            {
                /*
                * Print integer part
                */
                int_part = (int)(v);
                mul = 1;

                while (int_part / Base)
                {
                    mul *= Base;
                    int_part /= Base;
                }

                int_part = (int)(v);

                do
                {
                    _StoreChar(pBufferDesc, _aV2C[int_part / mul]);

                    if (pBufferDesc->ReturnValue < 0)
                    {
                        break;
                    }

                    int_part -= ((int)(int_part / mul)) * mul;
                    mul /= Base;
                }
                while (mul);

                /*
                * Print the symbol '.'
                */
                if (pBufferDesc->ReturnValue >= 0)
                {
                    _StoreChar(pBufferDesc, '.');

                    if (pBufferDesc->ReturnValue < 0)
                    {
                        ;
                    }

                    /*
                    * Print decimal part
                    */
                    if (pBufferDesc->ReturnValue >= 0)
                    {
                        mul = 1;
                        len = NumDigits;

                        while (len)
                        {
                            mul *= Base;
                            len--;
                        }

                        decimal_part = (int)((v - (int)v) * mul);

                        do
                        {
                            mul /= Base;
                            _StoreChar(pBufferDesc, _aV2C[decimal_part / mul]);

                            if (pBufferDesc->ReturnValue < 0)
                            {
                                ;
                            }

                            decimal_part -= ((int)(decimal_part / mul)) * mul;
                        }
                        while (mul / Base);
                    }
                }
            }

            /*
            * Print trailing zeros if necessary
            */

        }
    }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       ESSEMI_SWD_vprintf
*
*  Function description
*    Stores a formatted string in ESSEMI SWD control block.
*    This data is read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used. (e.g. 0 for "Terminal")
*    sFormat      Pointer to format string
*    pParamList   Pointer to the list of arguments for the format string
*
*  Return values
*    >= 0:  Number of bytes which have been stored in the "Up"-buffer.
*     < 0:  Error
*/
int ESSEMI_SWD_vprintf(unsigned BufferIndex, const char *sFormat, va_list *pParamList)
{
    char c;
    int v;
    double f_v;
    essemi_swd_printf_DESC BufferDesc;
    unsigned NumDigits;
    unsigned FormatFlags;
    unsigned FieldWidth;
    char acBuffer[essemi_swd_printf_BUFFER_SIZE];

    BufferDesc.pBuffer        = acBuffer;
    BufferDesc.BufferSize     = essemi_swd_printf_BUFFER_SIZE;
    BufferDesc.Cnt            = 0u;
    BufferDesc.SWDBufferIndex = BufferIndex;
    BufferDesc.ReturnValue    = 0;

    do
    {
        c = *sFormat;
        sFormat++;

        if (c == 0u)
        {
            break;
        }

        if (c == '%')
        {
            FormatFlags = 0u;
            v = 1;

            do
            {
                c = *sFormat;

                switch (c)
                {
                    case '-':
                        FormatFlags |= FORMAT_FLAG_LEFT_JUSTIFY;
                        sFormat++;
                        break;

                    case '0':
                        FormatFlags |= FORMAT_FLAG_PAD_ZERO;
                        sFormat++;
                        break;

                    case '+':
                        FormatFlags |= FORMAT_FLAG_PRINT_SIGN;
                        sFormat++;
                        break;

                    case '#':
                        FormatFlags |= FORMAT_FLAG_ALTERNATE;
                        sFormat++;
                        break;

                    default:
                        v = 0;
                        break;
                }
            }
            while (v);

            /*
            * filter out field width
            */
            FieldWidth = 0u;

            do
            {
                c = *sFormat;

                if ((c < '0') || (c > '9'))
                {
                    break;
                }

                sFormat++;
                FieldWidth = (FieldWidth * 10u) + ((unsigned)c - '0');
            }
            while (1);

            /*
            * Filter out precision (number of digits to display)
            */
            NumDigits = 0u;
            c = *sFormat;

            if (c == '.')
            {
                sFormat++;

                do
                {
                    c = *sFormat;

                    if ((c < '0') || (c > '9'))
                    {
                        break;
                    }

                    sFormat++;
                    NumDigits = NumDigits * 10u + ((unsigned)c - '0');
                }
                while (1);
            }

            /*
            * Filter out length modifier
            */
            c = *sFormat;

            do
            {
                if ((c == 'l') || (c == 'h'))
                {
                    sFormat++;
                    c = *sFormat;
                }
                else
                {
                    break;
                }
            }
            while (1);

            /*
            * Handle specifiers
            */
            switch (c)
            {
                case 'c':
                {
                    char c0;
                    v = va_arg(*pParamList, int);
                    c0 = (char)v;
                    _StoreChar(&BufferDesc, c0);
                    break;
                }

                case 'd':
                    v = va_arg(*pParamList, int);
                    _PrintInt(&BufferDesc, v, 10u, NumDigits, FieldWidth, FormatFlags);
                    break;

                case 'f':
                case 'F':
                    f_v = (double)va_arg(*pParamList, double);
                    _PrintFloat(&BufferDesc, f_v, 10u, NumDigits, FieldWidth, FormatFlags);
                    break;

                case 'u':
                    v = va_arg(*pParamList, int);
                    _PrintUnsigned(&BufferDesc, (unsigned)v, 10u, NumDigits, FieldWidth, FormatFlags);
                    break;

                case 'x':
                case 'X':
                    v = va_arg(*pParamList, int);
                    _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, NumDigits, FieldWidth, FormatFlags);
                    break;

                case 's':
                {
                    const char *s = va_arg(*pParamList, const char *);

                    do
                    {
                        c = *s;
                        s++;

                        if (c == '\0')
                        {
                            break;
                        }

                        _StoreChar(&BufferDesc, c);
                    }
                    while (BufferDesc.ReturnValue >= 0);
                }
                    break;

                case 'p':
                    v = va_arg(*pParamList, int);
                    _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, 8u, 8u, 0u);
                    break;

                case '%':
                    _StoreChar(&BufferDesc, '%');
                    break;

                default:
                    break;
            }

            sFormat++;
        }
        else
        {
            _StoreChar(&BufferDesc, c);
        }
    }
    while (BufferDesc.ReturnValue >= 0);

    if (BufferDesc.ReturnValue > 0)
    {
        //
        // Write remaining data, if any
        //
        if (BufferDesc.Cnt != 0u)
        {
            ESSEMI_SWD_Write(BufferIndex, acBuffer, BufferDesc.Cnt);
        }

        BufferDesc.ReturnValue += (int)BufferDesc.Cnt;
    }

    return BufferDesc.ReturnValue;
}

/*********************************************************************
*
*       essemi_swd_printf
*
*  Function description
*    Stores a formatted string in ESSEMI SWD control block.
*    This data is read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used. (e.g. 0 for "Terminal")
*    sFormat      Pointer to format string, followed by the arguments for conversion
*
*  Return values
*    >= 0:  Number of bytes which have been stored in the "Up"-buffer.
*     < 0:  Error
*
*  Notes
*    (1) Conversion specifications have following syntax:
*          %[flags][FieldWidth][.Precision]ConversionSpecifier
*    (2) Supported flags:
*          -: Left justify within the field width
*          +: Always print sign extension for signed conversions
*          0: Pad with 0 instead of spaces. Ignored when using '-'-flag or precision
*        Supported conversion specifiers:
*          c: Print the argument as one char
*          d: Print the argument as a signed integer
*          u: Print the argument as an unsigned integer
*          x: Print the argument as an hexadecimal integer
*          s: Print the string pointed to by the argument
*          p: Print the argument as an 8-digit hexadecimal integer. (Argument shall be a pointer to void.)
*/
int essemi_swd_printf(unsigned BufferIndex, const char *sFormat, ...)
{
    int r;
    va_list ParamList;

    va_start(ParamList, sFormat);
    r = ESSEMI_SWD_vprintf(BufferIndex, sFormat, &ParamList);
    va_end(ParamList);
    return r;
}
/*************************** End of file ****************************/
