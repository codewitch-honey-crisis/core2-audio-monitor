#pragma once
#include <gfx.hpp>
#include <uix.hpp>
using screen_t = uix::screen<gfx::rgb_pixel<16>>;
using surface_t = screen_t::control_surface_type;

extern screen_t main_screen;

template<typename ControlSurfaceType, size_t WindowSize = 512>
class analyzer_box : public uix::control<ControlSurfaceType> {
   public:
    using control_surface_type = ControlSurfaceType;
    using base_type = uix::control<control_surface_type>;
    using pixel_type = typename base_type::pixel_type;
    using palette_type = typename base_type::palette_type;
    using bitmap_type = gfx::bitmap<pixel_type, palette_type>;
    static constexpr const size_t window_size = WindowSize;
   private:
    float m_samples[window_size];
    float m_samples_buffer[window_size];
    float m_fft[window_size];
    float m_bar_chart[window_size];
    float m_bar_chart_peaks[window_size];
    bitmap_type m_spectrogram;
    gfx::rgb_pixel<16> m_palette[256];
    int m_state = 0;
    static void* psram_alloc(size_t size) {
        void* result =  heap_caps_malloc(size,MALLOC_CAP_SPIRAM);
        if(result==nullptr) {
            puts("PSRAM Heap alloc failed");
        }
        return result;
    }
   public:
    analyzer_box(uix::invalidation_tracker &parent, const palette_type *palette = nullptr)
        : base_type(parent, palette), m_spectrogram({0,0},nullptr) {
        for (int i = 0; i < window_size; i++) {
            m_bar_chart[i] = 0.0f;
        }
        for (int i = 0; i < window_size; i++) {
            m_bar_chart_peaks[i] = 0.0f;
        }
    }
    analyzer_box(analyzer_box &&rhs) {
        do_move_control(rhs);
    }
    const float *samples() const {
        return m_samples;
    }
    void samples(const float *values) {
        memcpy(m_samples, values, window_size * sizeof(float));
    }
    const float *fft() const {
        return m_fft;
    }
    void fft(const float *values) {
        memcpy(m_fft, values, window_size * sizeof(float));
    }
    analyzer_box &operator=(analyzer_box &&rhs) {
        do_move_control(rhs);
        return *this;
    }
    analyzer_box(const analyzer_box &rhs) {
        do_copy_control(rhs);
    }
    analyzer_box &operator=(const analyzer_box &rhs) {
        do_copy_control(rhs);
        return *this;
    }
    virtual ~analyzer_box() {
        if(m_spectrogram.begin()!=nullptr) {
            free(m_spectrogram.begin());
        }
    }
    virtual void on_before_render() {
        if (m_state == 0) {
            // init palette the first time
            gfx::rgb_pixel<24> px1(0, 0, 50), px2(0, 255, 0), px3(255, 220, 0), px4(255, 128, 0),
                px5(255, 0, 0);
            // init palette
            for (int i = 0; i < 128; ++i) {
                gfx::rgb_pixel<24> px24;
                px1.blend(px2, 1.0f - ((float)i / 128.0f), &px24);
                gfx::rgb_pixel<16> px16;
                gfx::convert(px24, &px16);
                m_palette[i] = px16;
            }
            for (int i = 128; i < 164; ++i) {
                gfx::rgb_pixel<24> px24;
                px2.blend(px3, 1.0f - ((float)(i-128) / 36.0f), &px24);
                gfx::rgb_pixel<16> px16;
                gfx::convert(px24, &px16);
                m_palette[i] = px16;
            }
            for (int i = 164; i < 192; ++i) {
                gfx::rgb_pixel<24> px24;
                px3.blend(px4, 1.0f - ((float)(i-164) / 28.0f), &px24);
                gfx::rgb_pixel<16> px16;
                gfx::convert(px24, &px16);
                m_palette[i] = px16;
            }
            for (int i = 192; i < 256; ++i) {
                gfx::rgb_pixel<24> px24;
                px4.blend(px5, 1.0f - ((float)(i-192) / 63.0f), &px24);
                gfx::rgb_pixel<16> px16;
                gfx::convert(px24, &px16);
                m_palette[i] = px16;
            }
            m_spectrogram=gfx::create_bitmap<pixel_type,palette_type>(gfx::size16(this->dimensions().width,this->dimensions().height/2),nullptr,psram_alloc);
            //m_spectrogram.fill(m_spectrogram.bounds(),pixel_type(0,0,6));
            m_state = 1;
        }
        memcpy(m_samples_buffer, m_samples, sizeof(m_samples_buffer));
        if(m_state==1) { // eq bars
            for (int i = 0; i < window_size; i++) {
                float m = m_fft[i];
                if (m > m_bar_chart[i]) {
                    m_bar_chart[i] = m;
                } else {
                    m_bar_chart[i] = 0.7f * m_bar_chart[i] + 0.3f * m;
                }
                if (m > m_bar_chart_peaks[i]) {
                    m_bar_chart_peaks[i] = m;
                } else {
                    m_bar_chart_peaks[i] = 0.95f * m_bar_chart_peaks[i] + 0.05f * m;
                }
            }
        } else if(m_state==2) { // spectroanalyzer
            static_assert(pixel_type::bit_depth==16,"this code needs to be ported for your display format");
            const size_t stride = m_spectrogram.dimensions().width * 2;
            uint8_t* p = m_spectrogram.begin();
            if(p!=nullptr) { // in case we didn't have memory
                // scroll left and put the new values in
                for(int y = 0; y<m_spectrogram.dimensions().height;++y) {
                    memmove(p,p+2,stride-2);
                    *(uint16_t*)&p[stride-2]=m_palette[std::max(0, std::min(255, (int)m_fft[m_spectrogram.dimensions().height-y-1]))].value();
                    p+=stride;
                }
            }
        }
    }
    virtual bool on_touch(size_t locations_size,const gfx::spoint16* locations) {
        return true;
    }
    virtual void on_release() override {
        ++m_state;
        if(m_state==2) {
            m_spectrogram.fill(m_spectrogram.bounds(),pixel_type(0,0,6));
        }
        if(m_state>2) {
            m_state = 1;
        }
    }
    virtual void on_paint(control_surface_type &destination, const gfx::srect16 &clip) override {
        if (clip.intersects(gfx::srect16(0, 0, destination.bounds().x2, destination.bounds().y2)
                                .offset(0, destination.dimensions().height / 2)))
        {
            if(m_state==1) {
                // eq bars
                int x = 0;
                int xi_step = int(float(destination.dimensions().width) / (window_size / 16));
                for (int i = 2; i < window_size / 4; i += 4) {
                    float ave = 0.0f;
                    for (int j = 0; j < 4; j++) {
                        ave += m_bar_chart[i + j];
                    }
                    ave *= .25f;
                    int bar_value = std::min(float(destination.dimensions().height/2), 0.25f * ave);
                    ave = 0;
                    for (int j = 0; j < 4; j++) {
                        ave += m_bar_chart_peaks[i + j];
                    }
                    ave *= .25f;
                    int peak_value = std::min(float(destination.dimensions().height/2), 0.25f * ave);
                    gfx::draw::line_aa(destination,gfx::srect16(x, destination.dimensions().height - peak_value, x+ xi_step - 1, destination.dimensions().height - peak_value), m_palette[std::max(0,std::min(255,peak_value+135))]); 
                    gfx::draw::filled_rectangle(destination,gfx::srect16(gfx::spoint16(x, destination.dimensions().height - bar_value),gfx::ssize16( xi_step - 1, bar_value)),m_palette[std::max(0,std::min(255,bar_value+135))]);
                    x += xi_step;
                }
            } else {
                // spectrogram
                gfx::rect16 clip2 = (gfx::rect16)clip.offset(0,-destination.dimensions().height/2);
                gfx::draw::bitmap(destination,(gfx::rect16)((gfx::srect16)m_spectrogram.bounds().offset(0,destination.dimensions().height/2)).crop(clip),m_spectrogram,clip2,gfx::bitmap_resize::crop,nullptr,nullptr);
            }
        }
        const float x_step = 4 * ((float)destination.dimensions().width / (float)window_size);
        const float y_offset = 60.0f;

        float sample_x = 0.0f;
        for (int i = 4; i < window_size; i += 4) {
            gfx::draw::line(destination,
                            gfx::rect16(sample_x,
                                        y_offset + m_samples_buffer[i - 4] * 3,
                                        sample_x + x_step,
                                        y_offset + m_samples_buffer[i] * 3),
                            gfx::rgb_pixel<16>(0xfff,true));
            sample_x += x_step;
        }
    }
};
using analyzer_box_t = analyzer_box<surface_t>;

extern analyzer_box_t main_analyzer;