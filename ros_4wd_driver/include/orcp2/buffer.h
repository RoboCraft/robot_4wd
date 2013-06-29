// 
// simple buffer object
//
// robocraft.ru
//

#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <stdio.h>
#include <string.h>

typedef struct CBuffer
{
	char *data;
	size_t size;
	size_t real_size;

	CBuffer(): data(0), size(0), real_size(0) {
	}

    inline void reset() {
		if(data) {
			delete []data;
			data = 0;
		}
		size = 0;
		real_size = 0;
	}

	inline void zero() 
	{
		if(data) {
			memset(data, 0, real_size);
			size = 0;
		}
	}

	inline void resize(size_t need_buf_size) {
		if(data && real_size >= need_buf_size)
			return;

		// delete old buffer
		reset();

		// allocate new buffer
		data = new char [need_buf_size];
		if(!data) {
			fprintf(stderr, "[!] Error: cant allocate memory!\n");
			return;
		}
		real_size = need_buf_size;	
	}

} CBuffer;

#ifdef __cplusplus

template <typename T>
class TBuff
{
public:
    TBuff():data(0), size(0), real_size(0) {
    }
    ~TBuff() {
        reset();
    }

    inline void reset() {
        if(data) {
            delete [] data;
            data = 0;
        }
        size = 0;
        real_size = 0;
    }

    inline void resize(size_t need_buf_size) {
        if(data && real_size >= need_buf_size)
            return;

        // delete old buffer
        reset();

        // allocate new buffer
        data = new T [need_buf_size];
        if(!data) {
            fprintf(stderr, "[!][TBuff] Error: cant allocate memory!\n");
            return;
        }
        real_size = need_buf_size;
    }

    inline void zero() {
        if(data) {
            memset(data, 0, real_size*sizeof(T));
            size = 0;
        }
    }

    T* data;
    size_t size;
    size_t real_size;
};

#endif //#ifdef __cplusplus

#endif //#ifndef _BUFFER_H_
