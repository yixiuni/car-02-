#include "gray_analog.h"
#include "car_config.h"
bool GrayAnalog_Decode(const uint8_t *b,size_t n,uint8_t command,uint16_t out[8])
{
    if (!b || !out || n!=21 || (command!=2 && command!=3) ||
        b[0]!=0x55 || b[1]!=0xAA || b[2]!=command || b[3]!=16) return false;
    uint8_t sum=0;
    for (size_t i=2;i<n;++i) sum=(uint8_t)(sum+b[i]);
    if (sum!=0xFF) return false;
    uint16_t values[8];
    for (size_t i=0;i<8;++i) values[i]=(uint16_t)(b[4+i*2]|((uint16_t)b[5+i*2]<<8));
    for (size_t i=0;i<8;++i) out[i]=values[i];
    return true;
}

void GrayAnalog_Init(volatile GrayAnalogReceiver *r,uint32_t now)
{
    *r=(GrayAnalogReceiver){0};r->activity_at=now;
}
bool GrayAnalog_CanRequest(const volatile GrayAnalogReceiver *r,uint32_t now)
{
    return !r->waiting && now-r->activity_at>=CAR_GRAY_QUERY_PERIOD_MS;
}
void GrayAnalog_BeginRequest(volatile GrayAnalogReceiver *r,uint32_t now)
{
    r->waiting=true;r->count=0;r->request_at=now;
}
void GrayAnalog_OnByte(volatile GrayAnalogReceiver *r,uint32_t now,uint8_t byte,bool error)
{
    r->activity_at=now;
    if(error){++r->errors;r->valid=r->waiting=false;return;}
    if(!r->waiting || now-r->request_at>=CAR_GRAY_RESPONSE_TIMEOUT_MS){
        ++r->unexpected_bytes;r->valid=r->waiting=false;return;
    }
    if(r->count>=21){++r->errors;r->valid=r->waiting=false;return;}
    r->bytes[r->count++]=byte;r->received_at=now;
}
void GrayAnalog_Poll(volatile GrayAnalogReceiver *r,uint32_t now)
{
    if(r->waiting && r->count==21 && now-r->received_at>=CAR_GRAY_SETTLE_MS){
        uint8_t bytes[21];uint16_t values[8];
        for(unsigned i=0;i<21;++i)bytes[i]=r->bytes[i];
        r->waiting=false;
        if(!GrayAnalog_Decode(bytes,21,2,values)){++r->errors;r->valid=false;return;}
        for(unsigned i=0;i<8;++i)r->values[i]=values[i];
        r->valid=true;r->last_valid_at=r->received_at;++r->samples;
    }else if(r->waiting && now-r->request_at>=CAR_GRAY_RESPONSE_TIMEOUT_MS){
        ++r->timeouts;r->valid=r->waiting=false;r->activity_at=now;
    }
}
uint8_t GrayAnalog_Classify(const uint16_t v[8],const uint16_t t[8],
    const uint16_t h[8],uint8_t high,uint8_t previous)
{
    uint8_t mask=previous;
    for(unsigned i=0;i<8;++i){
        int32_t difference=(int32_t)v[i]-(int32_t)t[i];
        if(!(high&(1U<<i)))difference=-difference;
        if(difference>(int32_t)h[i])mask|=(uint8_t)(1U<<i);
        else if(difference<-(int32_t)h[i])mask&=(uint8_t)~(1U<<i);
    }
    return mask;
}
