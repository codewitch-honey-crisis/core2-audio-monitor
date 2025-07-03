#pragma once

#include "assets/icons.h"
#include "palette.hpp"
static const auto waveform_color = gfx::rgb_pixel<16>(0xfff, true);
using screen_t = uix::screen<gfx::rgb_pixel<LCD_BIT_DEPTH>>;
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
    float m_min, m_max;
    float m_samples_buffer[window_size];
    float m_fft[window_size];
    float m_bar_chart[window_size];
    float m_bar_chart_peaks[window_size];
    bitmap_type m_spectrogram;
    int m_state = 0;
    gfx::font* m_fps_font;
    int m_fps;
    uix::srect16 m_fps_bounds;
    char m_fps_sz[16];
    gfx::text_info m_fps_ti;
    int m_power_level;
    bool m_power_ac;
    void update_fps_font() {
        if(m_fps_font!=nullptr) {
            m_fps_ti.text_font = m_fps_font;
            m_fps_ti.encoding = &gfx::text_encoding::utf8;
            itoa(m_fps,m_fps_sz,10);
            strcat(m_fps_sz," FPS");
            m_fps_ti.text_sz(m_fps_sz);
            gfx::size16 area;
            m_fps_font->measure(-1,m_fps_ti,&area);
            m_fps_bounds = gfx::srect16(this->dimensions().width-area.width-2,0,this->dimensions().width-1,area.height);
        }
    }
    void init() {
        for (int i = 0; i < window_size; i++) {
            m_bar_chart[i] = 0.0f;
        }
        for (int i = 0; i < window_size; i++) {
            m_bar_chart_peaks[i] = 0.0f;
        }
    }
   public:
    analyzer_box(uix::invalidation_tracker &parent, const palette_type *palette = nullptr)
        : base_type(parent, palette), m_spectrogram({0, 0}, nullptr), m_fps_font(nullptr), m_fps(0), m_power_level(0),m_power_ac(true) {
        init();
        
    }
    analyzer_box()
        : base_type(), m_spectrogram({0, 0}, nullptr), m_fps_font(nullptr), m_fps(0) ,m_power_level(0),m_power_ac(true){
        init();
    }
    analyzer_box(analyzer_box &&rhs) {
        m_state = 0;
        memcpy(m_samples, rhs.m_samples, sizeof(m_samples));
        memcpy(m_fft, rhs.m_fft, sizeof(m_fft));
        do_move_control(rhs);
    }
    gfx::font* fps_font() const {
        return m_fps_font;
    }
    void fps_font(gfx::font* value) {
        m_fps_font = value;
        update_fps_font();
    }
    int fps() const {
        return m_fps;
    }
    void fps(int value) {
        m_fps = value;
        update_fps_font();
    }
    const float *samples() const {
        return m_samples;
    }
    void samples(const float *values) {
        memcpy(m_samples, values, window_size * sizeof(float));
    }
    void extents(float min_val, float max_val) {
        m_min = min_val;
        m_max = max_val;
    }
    const float *fft() const {
        return m_fft;
    }
    void fft(const float *values) {
        memcpy(m_fft, values, window_size * sizeof(float));
    }
    int power_level() const { return m_power_level;}
    void power_level(int value) { 
        m_power_level = value;
    }
    bool power_ac() const { return m_power_ac;}
    void power_ac(bool value) { 
        m_power_ac = value;
    }
    
    analyzer_box &operator=(analyzer_box &&rhs) {
        m_state = 0;
        memcpy(m_samples, rhs.m_samples, sizeof(m_samples));
        memcpy(m_fft, rhs.m_fft, sizeof(m_fft));
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
        if (m_spectrogram.begin() != nullptr) {
            free(m_spectrogram.begin());
        }
    }
    virtual void on_before_paint() override {
        if (m_state == 0) {
            // create the spectrogram bitmap
            m_spectrogram = gfx::create_bitmap<pixel_type, palette_type>(
                gfx::size16(this->dimensions().width, this->dimensions().height / 2));
            m_state = 1;
        }
        memcpy(m_samples_buffer, m_samples, sizeof(m_samples_buffer));
        if (m_state == 1) {  // eq bars
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
        } else if (m_state == 2) {  // spectroanalyzer
            static_assert(pixel_type::template equals<gfx::rgb_pixel<16>>::value,
                          "this code needs to be ported for your display format");
            typename screen_t::pixel_type mapped;
            // scroll left and put the new values in
            gfx::blt_span& bs = m_spectrogram;
            switch(bs.pixel_width()) {
                case 2:
                    for (int y = 0; y < m_spectrogram.dimensions().height; ++y) {
                        gfx::gfx_span s = bs.span(gfx::point16(0,y));
                        memmove(s.data, s.data + bs.pixel_width(), s.length - bs.pixel_width());;
                        analyzer_palette<typename screen_t::pixel_type>::instance.map(
                            gfx::helpers::clamp((int)m_fft[m_spectrogram.dimensions().height - y - 1],0,255),&mapped);
#ifndef HTCW_GFX_NO_SWAP
                        *(uint16_t *)&s.data[s.length - bs.pixel_width()] = mapped.swapped();
#else
                        *(uint16_t *)&s.data[s.length - bs.pixel_width()] = mapped;
#endif
                    }
                    break;
                case 4:
                    for (int y = 0; y < m_spectrogram.dimensions().height; ++y) {
                        gfx::gfx_span s = bs.span(gfx::point16(0,y));
                        memmove(s.data, s.data + bs.pixel_width(), s.length - bs.pixel_width());;
                        analyzer_palette<typename screen_t::pixel_type>::instance.map(
                            gfx::helpers::clamp((int)m_fft[m_spectrogram.dimensions().height - y - 1],0,255),&mapped);
#ifndef HTCW_GFX_NO_SWAP
                        *(uint32_t *)&s.data[s.length - bs.pixel_width()] = mapped.swapped();
#else
                        *(uint32_t *)&s.data[s.length - bs.pixel_width()] = mapped;
#endif
                    }
                    break;
                case 1:
                    for (int y = 0; y < m_spectrogram.dimensions().height; ++y) {
                        gfx::gfx_span s = bs.span(gfx::point16(0,y));
                        memmove(s.data, s.data + bs.pixel_width(), s.length - bs.pixel_width());;
                        analyzer_palette<typename screen_t::pixel_type>::instance.map(
                            gfx::helpers::clamp((int)m_fft[m_spectrogram.dimensions().height - y - 1],0,255),&mapped);
#ifndef HTCW_GFX_NO_SWAP
                        s.data[s.length - bs.pixel_width()] = mapped.swapped();
#else
                        s.data[s.length - bs.pixel_width()] = mapped;
#endif
                    }
                    break;
                case 8:
                    for (int y = 0; y < m_spectrogram.dimensions().height; ++y) {
                        gfx::gfx_span s = bs.span(gfx::point16(0,y));
                        memmove(s.data, s.data + bs.pixel_width(), s.length - bs.pixel_width());;
                        analyzer_palette<typename screen_t::pixel_type>::instance.map(
                            gfx::helpers::clamp((int)m_fft[m_spectrogram.dimensions().height - y - 1],0,255),&mapped);
#ifndef HTCW_GFX_NO_SWAP
                        *(uint64_t *)&s.data[s.length - bs.pixel_width()] = mapped.swapped();
#else
                        *(uint64_t *)&s.data[s.length - bs.pixel_width()] = mapped;
#endif
                    }
                    break;
            
            }
        }
    }
    virtual bool on_touch(size_t locations_size, const gfx::spoint16 *locations) {
        return true;
    }
    virtual void on_release() override {
        ++m_state;
        if (m_state == 2) {
            m_spectrogram.fill(m_spectrogram.bounds(),screen_t::pixel_type(0,true));
        }
        if (m_state > 2) {
            m_state = 1;
        }
    }
    virtual void on_paint(control_surface_type &destination, const gfx::srect16 &clip) override {
        if (clip.intersects(gfx::srect16(0, 0, destination.bounds().x2, destination.bounds().y2)
                                .offset(0, destination.dimensions().height / 2)))
        {
            if (m_state == 1) {
                // eq bars
                int x = 0;
                int xi_step = int(float(destination.dimensions().width) / (window_size / 16));
                for (int i = 2; i < window_size / 4; i += 4) {
                    float ave = 0.0f;
                    for (int j = 0; j < 4; j++) {
                        ave += m_bar_chart[i + j];
                    }
                    ave *= .25f;
                    int bar_value = std::min(float(destination.dimensions().height / 2),
                                             0.25f * ave);
                    ave = 0;
                    for (int j = 0; j < 4; j++) {
                        ave += m_bar_chart_peaks[i + j];
                    }
                    ave *= .25f;
                    int peak_value = std::min(float(destination.dimensions().height / 2),
                                              0.25f * ave);
                    typename screen_t::pixel_type mapped;
                    analyzer_palette<typename screen_t::pixel_type>::instance.map(
                        gfx::helpers::clamp(peak_value + 135, 0, 255), &mapped);
                    gfx::draw::line(destination,
                                       gfx::srect16(x,
                                                    destination.dimensions().height - peak_value,
                                                    x + xi_step - 1,
                                                    destination.dimensions().height - peak_value),
                                       mapped);
                    analyzer_palette<typename screen_t::pixel_type>::instance.map(
                        gfx::helpers::clamp(bar_value + 135, 0, 255), &mapped);
                    gfx::draw::filled_rectangle(
                        destination,
                        gfx::srect16(gfx::spoint16(x, destination.dimensions().height - bar_value),
                                     gfx::ssize16(xi_step - 1, bar_value)),
                        mapped);
                    x += xi_step;
                }
            } else {
                // spectrogram
                gfx::rect16 clip2 = (gfx::rect16)clip.offset(0,
                                                             -destination.dimensions().height / 2);
                gfx::draw::bitmap(destination,
                                  (gfx::rect16)((gfx::srect16)m_spectrogram.bounds().offset(
                                                    0, destination.dimensions().height / 2))
                                      .crop(clip),
                                  m_spectrogram,
                                  clip2);
            }
        }
        constexpr static const gfx::rgb_pixel<16> white(0xFFFF,true);
        if(m_fps_font!=nullptr && clip.intersects(m_fps_bounds)) {
            gfx::draw::text(destination,m_fps_bounds,m_fps_ti,white);
        }
            // show in green if it's on ac power.
        constexpr static const gfx::rgb_pixel<16> green(0,63,0);
        constexpr static const gfx::rgb_pixel<16> red(31,0,0);
        if(!m_power_ac) {
            if(clip.intersects(gfx::srect16(0,0,27,24))) {
                auto px = white;
                if(!m_power_ac && m_power_level<25) {
                    px=red;
                }   
                // draw an empty battery
                gfx::const_bitmap<gfx::alpha_pixel<8>> cico({27,24},faBatteryEmpty);
                gfx::draw::icon(destination,gfx::spoint16::zero(),cico,px);
                // now fill it up
                if(m_power_level==100) {
                    // if we're at 100% fill the entire thing
                    gfx::draw::filled_rectangle(destination,gfx::srect16(3,7,22,16),px);
                } else {
                    // otherwise leave a small border
                    gfx::draw::filled_rectangle(destination,gfx::srect16(4,9,4+(0.18f*m_power_level),14),px);
                }
            }
        }
        const float x_step = 4 * ((float)destination.dimensions().width / (float)window_size);
        const float y_offset = 60.0f;

        float sample_x = 0.0f;
        for (int i = 4; i < window_size; i += 4) {
            gfx::draw::line(destination,
                            gfx::srect16(sample_x,
                                        y_offset + m_samples_buffer[i - 4] * 3,
                                        sample_x + x_step,
                                        y_offset + m_samples_buffer[i] * 3),
                            waveform_color);
            sample_x += x_step;
        }
    
    }
};
using analyzer_box_t = analyzer_box<typename screen_t::control_surface_type>;

extern analyzer_box_t main_analyzer;