#pragma once

#include <stddef.h>

#define use_phys 0
#define physaddrbase 0

unsigned int memtester_stm32(void* ram, size_t bufsize, size_t loops);

unsigned int rand32();
