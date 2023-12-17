#include "new.h"
#include <new>
#include <stdlib.h>
#include <typeinfo>
#include <algorithm>
#include <string.h>

//#define PROFILE_MEMORY

#ifdef PROFILE_MEMORY

struct TypeInfo {
	void* vtable;
	const char* name;
};

// set print array on
// set print elements 0
// p aggregatedAllocInfo
struct Vtable {
	void* entries[1];
};

struct Object {
	Vtable* vtable;
};

struct AllocInfo {
	void* address;
	unsigned int size;
	void* caller;
	const char* type_name;
};

AllocInfo allocInfo[4500];
unsigned int allocInfoIndex;
unsigned int allocInfoTotal;


struct AllocInfoAggregated {
	void* address;
	unsigned int size;
	unsigned int count;
	const char* type_name;
	void* caller;
};


AllocInfoAggregated aggregatedAllocInfo[100];
unsigned int aggregatedAllocInfoIndex;
unsigned int aggregatedAllocInfoTotal;
unsigned int aggregatedAllocInfoUsedNumber;

void * operator new(std::size_t n)
{
	void* memory = malloc(n);
	if(allocInfoIndex < sizeof(allocInfo)/sizeof(allocInfo[0])) {
		allocInfo[allocInfoIndex].caller = __builtin_return_address(0);
		allocInfo[allocInfoIndex].address = memory;
		allocInfo[allocInfoIndex].size = n;
		allocInfoIndex++;
	}
	allocInfoTotal++;

	return memory;
}
void operator delete(void * p)
{
	free(p);
	for(unsigned int i = 0; i < allocInfoIndex; i++) {
		if(allocInfo[i].address == p) {
			allocInfo[i].address = nullptr;
			allocInfo[i].caller = nullptr;
			allocInfo[i].size = 0;
			break;
		}
	}
}

static void updateTypeNames(void) {
	for(unsigned int i = 0; i < allocInfoIndex; i++) {
		if(allocInfo[i].address != nullptr) {
			Object* object = (Object*)allocInfo[i].address;
			if(object->vtable < ((void*)0x08000000) || object->vtable >= ((void*)0x08ffffff)) {
				// Vtable must be in FLASH
				continue;
			}

			if(object->vtable->entries[-1] < ((void*)0x08000000) || object->vtable->entries[-1] >= ((void*)0x08ffffff)) {
				// typeinfo must be in FLASH
				continue;
			}
			TypeInfo* typeinfo = (TypeInfo*)object->vtable->entries[-1];
			allocInfo[i].type_name = typeinfo->name;
		}
	}
}

void generateStatistics(void) {
	if(aggregatedAllocInfoUsedNumber == allocInfoIndex)
		return;

	updateTypeNames();

	aggregatedAllocInfoUsedNumber = allocInfoIndex;

	aggregatedAllocInfoIndex = 0;
	aggregatedAllocInfoTotal = 0;
	memset(aggregatedAllocInfo, 0, sizeof(aggregatedAllocInfo));

	for(unsigned int i = 0; i < allocInfoIndex; i++) {
		bool existing = false;

		if(allocInfo[i].address == 0)
			continue;

		for(unsigned int j = 0; j < aggregatedAllocInfoIndex; j++) {
			if(aggregatedAllocInfo[j].type_name == allocInfo[i].type_name &&
				aggregatedAllocInfo[j].caller == allocInfo[i].caller) {
				existing = true;

				aggregatedAllocInfo[j].size += allocInfo[i].size;
				aggregatedAllocInfo[j].count++;
				break;
			}
		}
		if(!existing) {
			if(aggregatedAllocInfoIndex < sizeof(aggregatedAllocInfo)/sizeof(aggregatedAllocInfo[0])) {
				aggregatedAllocInfo[aggregatedAllocInfoIndex].address = allocInfo[i].address;
				aggregatedAllocInfo[aggregatedAllocInfoIndex].caller = allocInfo[i].caller;
				aggregatedAllocInfo[aggregatedAllocInfoIndex].size = allocInfo[i].size;
				aggregatedAllocInfo[aggregatedAllocInfoIndex].count = 1;
				aggregatedAllocInfo[aggregatedAllocInfoIndex].type_name = allocInfo[i].type_name;

				aggregatedAllocInfoIndex++;
			}
			aggregatedAllocInfoTotal++;
		}
	}

    struct
    {
        bool operator()(const AllocInfoAggregated& a, const AllocInfoAggregated& b) const {
			return a.size > b.size;
		}
    } customLess;
	std::sort(aggregatedAllocInfo, aggregatedAllocInfo + aggregatedAllocInfoIndex, customLess);
}

#else
void generateStatistics(void) {}
#endif