#ifndef __LIBDEPTHSENSE_DS_HPP__
#define __LIBDEPTHSENSE_DS_HPP__

#include "ds.h"

#include <string>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <exception>

namespace ds
{
    typedef ds_stream       stream;
    typedef ds_format       format;
    typedef ds_preset       preset;
    typedef ds_distortion   distortion;
    typedef ds_option       option;
    typedef ds_intrinsics   intrinsics;
    typedef ds_extrinsics   extrinsics;
    typedef ds_vte_data     vte_data;
    typedef ds_vte_param    vte_param;
    
    struct Size
    {
        Size() : width(0), height(0) {}
        Size(int _w, int _h) : width(_w), height(_h) {}

        int width;
        int height;
    };

    template<typename _Tp>
    struct Rect_
    {
        Rect_() : x(0), y(0), width(0), height(0) {}
        Rect_(_Tp _x, _Tp _y, _Tp _w, _Tp _h) : x(_x), y(_y), width(_w), height(_h) {}

        _Tp x;
        _Tp y;
        _Tp width;
        _Tp height;

        // area
        _Tp area() const
        {
            return width * height;
        }
    };

    template<typename _Tp> static inline Rect_<_Tp>& operator &= ( Rect_<_Tp>& a, const Rect_<_Tp>& b )
    {
        _Tp x1 = std::max(a.x, b.x), y1 = std::max(a.y, b.y);
        a.width = std::min(a.x + a.width, b.x + b.width) - x1;
        a.height = std::min(a.y + a.height, b.y + b.height) - y1;
        a.x = x1; a.y = y1;
        if( a.width <= 0 || a.height <= 0 )
            a = Rect_<_Tp>();
        return a;
    }

    template<typename _Tp> static inline Rect_<_Tp>& operator |= ( Rect_<_Tp>& a, const Rect_<_Tp>& b )
    {
        _Tp x1 = std::min(a.x, b.x), y1 = std::min(a.y, b.y);
        a.width = std::max(a.x + a.width, b.x + b.width) - x1;
        a.height = std::max(a.y + a.height, b.y + b.height) - y1;
        a.x = x1; a.y = y1;
        return a;
    }

    template<typename _Tp> static inline Rect_<_Tp> operator & (const Rect_<_Tp>& a, const Rect_<_Tp>& b)
    {
        Rect_<_Tp> c = a;
        return c &= b;
    }

    template<typename _Tp> static inline Rect_<_Tp> operator | (const Rect_<_Tp>& a, const Rect_<_Tp>& b)
    {
        Rect_<_Tp> c = a;
        return c |= b;
    }

    typedef Rect_<int> Rect;
    typedef Rect_<float> Rect2f;

    template<typename _Tp>
    struct Point_
    {
        Point_() : x(0), y(0) {}
        Point_(_Tp _x, _Tp _y) : x(_x), y(_y) {}

        _Tp x;
        _Tp y;
    };

    typedef Point_<int> Point;
    typedef Point_<float> Point2f;

#define DS_8UC1  1
#define DS_8UC2  2
#define DS_8UC3  3
#define DS_8UC4  4

#define DS_16SC1 2
#define DS_16SC2 4

#define DS_32FC1 4
#define DS_32FC2 8
#define DS_32FC3 12
#define DS_32FC4 16

#define DS_64FC1 8
#define DS_64FC2 16
#define DS_64FC3 24
#define DS_64FC4 32

// exchange-add operation for atomic operations on reference counters
#if defined __INTEL_COMPILER && !(defined WIN32 || defined _WIN32)
// atomic increment on the linux version of the Intel(tm) compiler
#  define DS_XADD(addr, delta) (int)_InterlockedExchangeAdd(const_cast<void*>(reinterpret_cast<volatile void*>(addr)), delta)
#elif defined __GNUC__
#  if defined __clang__ && __clang_major__ >= 3 && !defined __ANDROID__ && !defined __EMSCRIPTEN__ && !defined(__CUDACC__)
#    ifdef __ATOMIC_ACQ_REL
#      define DS_XADD(addr, delta) __c11_atomic_fetch_add((_Atomic(int)*)(addr), delta, __ATOMIC_ACQ_REL)
#    else
#      define DS_XADD(addr, delta) __atomic_fetch_add((_Atomic(int)*)(addr), delta, 4)
#    endif
#  else
#    if defined __ATOMIC_ACQ_REL && !defined __clang__
// version for gcc >= 4.7
#      define DS_XADD(addr, delta) (int)__atomic_fetch_add((unsigned*)(addr), (unsigned)(delta), __ATOMIC_ACQ_REL)
#    else
#      define DS_XADD(addr, delta) (int)__sync_fetch_and_add((unsigned*)(addr), (unsigned)(delta))
#    endif
#  endif
#elif defined _MSC_VER && !defined RC_INVOKED
#  include <intrin.h>
#  define DS_XADD(addr, delta) (int)_InterlockedExchangeAdd((long volatile*)addr, delta)
#else
    static inline void DS_XADD(int* addr, int delta) { int tmp = *addr; *addr += delta; return tmp; }
#endif

// the alignment of all the allocated buffers
#define MALLOC_ALIGN    16
    // Aligns a pointer to the specified number of bytes
    // ptr Aligned pointer
    // n Alignment size that must be a power of two
    template<typename _Tp> static inline _Tp* alignPtr(_Tp* ptr, int n=(int)sizeof(_Tp))
    {
        return (_Tp*)(((size_t)ptr + n-1) & -n);
    }


    static inline void* fastMalloc(size_t size)
    {
        unsigned char* udata = (unsigned char*)malloc(size + sizeof(void*) + MALLOC_ALIGN);
        if (!udata)
            return 0;
        unsigned char** adata = alignPtr((unsigned char**)udata + 1, MALLOC_ALIGN);
        adata[-1] = udata;
        return adata;
    }

    static inline void fastFree(void* ptr)
    {
        if (ptr)
        {
            unsigned char* udata = ((unsigned char**)ptr)[-1];
            free(udata);
        }
    }


    struct Mat
    {
        Mat() : data(0), refcount(0), rows(0), cols(0), c(0), frame_num(0), timestamp(0) {}

        Mat(int _rows, int _cols, int flags) : data(0), refcount(0), frame_num(0), timestamp(0)
        {
            create(_rows, _cols, flags);
        }

        // copy
        Mat(const Mat& m) : data(m.data), refcount(m.refcount)
        {
            if (refcount)
                DS_XADD(refcount, 1);

            rows = m.rows;
            cols = m.cols;
            c = m.c;
            frame_num = m.frame_num;
            timestamp = m.timestamp;
        }

        Mat(int _rows, int _cols, int flags, void* _data) : data((unsigned char*)_data), refcount(0), frame_num(0), timestamp(0)
        {
            rows = _rows;
            cols = _cols;
            c    = flags;
        }
        
        Mat(int _rows, int _cols, int flags, void* _data, unsigned long _frame_num, unsigned long _timestamp) : data((unsigned char*)_data), refcount(0)
        {
            rows      = _rows;
            cols      = _cols;
            c         = flags;
            frame_num = _frame_num;
            timestamp = _timestamp;
        }
        

        ~Mat()
        {
            release();
        }

        // assign
        Mat& operator=(const Mat& m)
        {
            if (this == &m)
                return *this;

            if (m.refcount)
                DS_XADD(m.refcount, 1);

            release();

            data = m.data;
            refcount = m.refcount;

            rows = m.rows;
            cols = m.cols;
            c = m.c;
            frame_num = m.frame_num;
            timestamp = m.timestamp;

            return *this;
        }

        void create(int _rows, int _cols, int flags)
        {
            release();

            rows = _rows;
            cols = _cols;
            c = flags;

            if (total() > 0)
            {
                // refcount address must be aligned, so we expand totalsize here
                size_t totalsize = (total() + 3) >> 2 << 2;
                data = (unsigned char*)fastMalloc(totalsize + (int)sizeof(*refcount));
                refcount = (int*)(((unsigned char*)data) + totalsize);
                *refcount = 1;
            }
        }

        void release()
        {
            if (refcount && DS_XADD(refcount, -1) == 1) {
                fastFree(data);
            }

            data = 0;

            rows = 0;
            cols = 0;
            c = 0;

            frame_num = 0;
            timestamp = 0;

            refcount = 0;
        }

        Mat clone() const
        {
            if (empty())
                return Mat();

            Mat m(rows, cols, c);

            if (total() > 0)
            {
                memcpy(m.data, data, total());
            }

            return m;
        }

        bool empty() const { return data == 0 || total() == 0; }

        int channels() const { return c; }

        // total size in bytes
        size_t total() const { return cols * rows * c; }

        size_t element_size(void) { return c; }

        const unsigned char* ptr(int y) const { return data + y * cols * c; }

        unsigned char* ptr(int y) { return data + y * cols * c; }

        unsigned long frame_number() { return frame_num; }
        unsigned long timestamp_ms() { return timestamp; }

        // roi
        Mat operator()( const Rect& roi ) const
        {
            if (empty())
                return Mat();

            Mat m(roi.height, roi.width, c);

            int sy = roi.y;
            for (int y = 0; y < roi.height; y++)
            {
                const unsigned char* sptr = ptr(sy) + roi.x * c;
                unsigned char* dptr = m.ptr(y);
                memcpy(dptr, sptr, roi.width * c);
                sy++;
            }

            return m;
        }

        unsigned char* data;

        // pointer to the reference counter;
        // when points to user-allocated data, the pointer is NULL
        int* refcount;

        int rows;
        int cols;

        int c;

        unsigned long frame_num;
        unsigned long timestamp;    //timestamp in ms
    }; // end of struct Mat
    
    class error : public std::runtime_error
    {
        std::string function, args;
        ds_exception_type type;
    public:
        explicit error(ds_error* err) : runtime_error(ds_get_error_message(err))
        {
            function = (nullptr != ds_get_failed_function(err)) ? ds_get_failed_function(err) : std::string();
            args = (nullptr != ds_get_failed_args(err)) ? ds_get_failed_args(err) : std::string();
            type = ds_get_libdepthsense_exception_type(err);
            ds_free_error(err);
        }

        explicit error(const std::string& message) : runtime_error(message.c_str())
        {
            function = "";
            args = "";
            type = DS_EXCEPTION_TYPE_UNKNOWN;
        }

        const std::string& get_failed_function() const
        {
            return function;
        }

        const std::string& get_failed_args() const
        {
            return args;
        }

        ds_exception_type get_type() const { return type; }

        static void handle(ds_error* e);
    };
    
    #define DS_ERROR_CLASS(name, base) \
    class name : public base\
    {\
    public:\
        explicit name(ds_error* e) noexcept : base(e) {}\
    }

    DS_ERROR_CLASS(recoverable_error, error);
    DS_ERROR_CLASS(unrecoverable_error, error);
    DS_ERROR_CLASS(camera_disconnected_error, unrecoverable_error);
    DS_ERROR_CLASS(backend_error, unrecoverable_error);
    DS_ERROR_CLASS(device_in_recovery_mode_error, unrecoverable_error);
    DS_ERROR_CLASS(invalid_value_error, recoverable_error);
    DS_ERROR_CLASS(wrong_api_call_sequence_error, recoverable_error);
    DS_ERROR_CLASS(not_implemented_error, recoverable_error);
    #undef DS_ERROR_CLASS

    inline void error::handle(ds_error* e)
    {
        if (e)
        {
            auto h = ds_get_libdepthsense_exception_type(e);
            switch (h) {
            case DS_EXCEPTION_TYPE_CAMERA_DISCONNECTED:
                throw camera_disconnected_error(e);
            case DS_EXCEPTION_TYPE_BACKEND:
                throw backend_error(e);
            case DS_EXCEPTION_TYPE_INVALID_VALUE:
                throw invalid_value_error(e);
            case DS_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE:
                throw wrong_api_call_sequence_error(e);
            case DS_EXCEPTION_TYPE_NOT_IMPLEMENTED:
                throw not_implemented_error(e);
            case DS_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE:
                throw device_in_recovery_mode_error(e);
            default:
                throw error(e);
            }
        }
    }
    
    // Modify this function if you wish to change error handling behavior from throwing exceptions to logging / callbacks / global status, etc.
    inline void handle_error(ds_error * error)
    {
#if 0
        std::ostringstream ss;
        ss << ds_get_error_message(error) << " (" << ds_get_failed_function(error) << '(' << ds_get_failed_args(error) << "))";
        ds_free_error(error);
        throw std::runtime_error(ss.str());
#else
        error::handle(error);
#endif
        // std::cerr << ss.str() << std::endl;
    }
    
    class auto_error
    {
    public:
                            auto_error()                                                            : error() {}
                            auto_error(const auto_error &)                                          = delete;
                            ~auto_error() noexcept(false)                                           { if (error) handle_error(error); }
        auto_error &        operator = (const auto_error &)                                         = delete;
                            operator ds_error ** ()                                                 { return &error; }
    private:
        ds_error *          error;
    };


    inline const char *     get_name(stream s)                                                      { return ds_get_stream_name(s, auto_error()); }
    inline const char *     get_name(format f)                                                      { return ds_get_format_name(f, auto_error()); }
    inline const char *     get_name(preset p)                                                      { return ds_get_preset_name(p, auto_error()); }
    inline const char *     get_name(distortion d)                                                  { return ds_get_distortion_name(d, auto_error()); }
    inline const char *     get_name(option d)                                                      { return ds_get_option_name(d, auto_error()); }

    class camera
    {
    private:
        ds_camera *         cam;
    public:
                            camera()                                                                : cam() {}
                            camera(ds_camera * cam)                                                 : cam(cam) {}
        explicit            operator bool() const                                                   { return !!cam; }
        ds_camera *         get_handle() const                                                      { return cam; }

        const char *        get_name()                                                              { return ds_get_camera_name(cam, auto_error()); }
        const char *        get_sn()                                                                { return ds_get_camera_sn(cam, auto_error()); }
        const char *        get_fw_version()                                                        { return ds_get_fw_version(cam, auto_error()); }
        const char *        get_device_path()                                                       { return ds_get_device_path(cam, auto_error()); }

        void                enable_stream(stream s, int width, int height, format f, int fps)       { ds_enable_stream(cam, s, width, height, f, fps, auto_error()); }
        void                enable_stream_preset(stream s, preset preset)                           { ds_enable_stream_preset(cam, s, preset, auto_error()); }
        bool                is_stream_enabled(stream s)                                             { return !!ds_is_stream_enabled(cam, s, auto_error()); }

        void                start_capture()                                                         { ds_start_capture(cam, auto_error()); }
        void                stop_capture()                                                          { ds_stop_capture(cam, auto_error()); }
        bool                is_capturing()                                                          { return !!ds_is_capturing(cam, auto_error()); }

        float               get_depth_scale()                                                       { return ds_get_depth_scale(cam, auto_error()); }
        int                 get_stream_width(stream s)                                              { return ds_get_stream_width(cam, s, auto_error()); }
        int                 get_stream_height(stream s)                                             { return ds_get_stream_height(cam, s, auto_error()); }
        format              get_stream_format(stream s)                                             { return ds_get_stream_format(cam, s, auto_error()); }
        intrinsics          get_stream_intrinsics(stream s)                                         { intrinsics intrin; ds_get_stream_intrinsics(cam, s, &intrin, auto_error()); return intrin; }
        extrinsics          get_stream_extrinsics(stream from, stream to)                           { extrinsics extrin; ds_get_stream_extrinsics(cam, from, to, &extrin, auto_error()); return extrin; }

        void                wait_all_streams(int ms = 500)                                          { ds_wait_all_streams(cam, ms, auto_error()); }
        void                poll_all_streams()                                                      { ds_poll_all_streams(cam, auto_error()); }
        const void *        get_image_pixels(stream s)                                              { return ds_get_image_pixels(cam, s, auto_error()); }
        const void *        get_image_vte_data(stream s, unsigned int *count)                       { return ds_get_image_vte_data(cam, s, count, auto_error()); }

        Mat                 get_image_mat(stream s) 
                            {
                                Mat m = Mat(get_stream_height(s), get_stream_width(s), get_stream_format(s), (void *)get_image_pixels(s), 
                                        get_image_frame_number(s), get_image_timestamp(s));
                                return m;
                            }

        unsigned long       get_image_frame_number(stream s)                                        { return ds_get_image_frame_number(cam, s, auto_error()); }
        unsigned long       get_image_timestamp(stream s)                                           { return ds_get_image_timestamp(cam, s, auto_error()); }

        bool                supports_option(option o) const                                         { return ds_camera_supports_option(cam, o, auto_error()); }
        void                set_option(option o, const void *buf, int bytes)                        { ds_set_camera_option(cam, o, buf, bytes, auto_error()); }
        void                get_option(option o, void *buf, int *bytes)                             { return ds_get_camera_option(cam, o,buf, bytes, auto_error()); }
    };

    class context
    {
    private:
        ds_context *        ctx;
    public:
                            context()                                                               : ctx(ds_create_context(DS_API_VERSION, auto_error())) {}
                            context(const context &)                                                = delete;
                            ~context()                                                              { ds_delete_context(ctx, nullptr); } // Deliberately ignore error on destruction
                            context & operator = (const context &)                                  = delete;
        ds_context *        get_handle() const                                                      { return ctx; }

        int                 get_camera_count()                                                      { return ds_get_camera_count(ctx, auto_error()); }
        camera              get_camera(int index)                                                   { return ds_get_camera(ctx, index, auto_error()); }
        void                refresh_devices()                                                       { ds_refresh_devices(ctx, auto_error()); }
    };

}

inline std::ostream & operator << (std::ostream & out, ds::stream stream)         { return out << ds::get_name(stream); }
inline std::ostream & operator << (std::ostream & out, ds::format format)         { return out << ds::get_name(format); }
inline std::ostream & operator << (std::ostream & out, ds::preset preset)         { return out << ds::get_name(preset); }
inline std::ostream & operator << (std::ostream & out, ds::distortion distortion) { return out << ds::get_name(distortion); }
inline std::ostream & operator << (std::ostream & out, ds::option option)         { return out << ds::get_name(option); }

#endif
