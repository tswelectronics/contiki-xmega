/*
 * Copyright (c) 2010, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: uip_arch.c,v 1.1 2010/10/25 09:03:39 salvopitru Exp $
 */
/*---------------------------------------------------------------------------*/
/**
* \file
*			uip_arch.c
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/
/*---------------------------------------------------------------------------*/




#include "net/uip.h"
#include "net/uip_arch.h"

/*-----------------------------------------------------------------------------------*/
#if UIP_TCP
void
uip_add32(u8_t *op32, u16_t op16)
{
  uint32_t op32_align, uip_acc32_align;
  
  op32_align = ((uint32_t)op32[0])<<24 | ((uint32_t)op32[1])<<16 | ((uint16_t)op32[2])<<8 | op32[3];
  
  uip_acc32_align = op32_align + op16;
  
  uip_acc32[3] = uip_acc32_align;
  uip_acc32[2] = uip_acc32_align>>8;
  uip_acc32[1] = uip_acc32_align>>16;
  uip_acc32[0] = uip_acc32_align>>24;  
  
}
#endif
/*-----------------------------------------------------------------------------------*/
