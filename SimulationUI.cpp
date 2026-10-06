#include "SimulationUI.h"
#include <QApplication>
#include <QPalette>
#include <cmath>
#include <initializer_list>

namespace {
static double parseDoubleOrDefault(const QLineEdit* edit, double default_value) {
    if (!edit) {
        return default_value;
    }

    bool ok = false;
    const double value = edit->text().toDouble(&ok);
    return ok && std::isfinite(value) ? value : default_value;
}

static double safeCheckedButtonValue(const QButtonGroup* group, int default_id) {
    if (!group) {
        return static_cast<double>(default_id);
    }

    const int checked = group->checkedId();
    return checked >= 0 ? static_cast<double>(checked) : static_cast<double>(default_id);
}

static void updateVerticalAxis(QValueAxis* axis, std::initializer_list<QLineSeries*> series,
                               double min_x, double max_x) {
    if (!axis) {
        return;
    }

    double min_y = 0.0;
    double max_y = 0.0;
    bool has_value = false;
    for (const QLineSeries* line : series) {
        if (!line) {
            continue;
        }

        for (const QPointF& point : line->points()) {
            if (point.x() < min_x || point.x() > max_x || !std::isfinite(point.y())) {
                continue;
            }

            if (!has_value) {
                min_y = point.y();
                max_y = point.y();
                has_value = true;
            } else {
                min_y = std::min(min_y, point.y());
                max_y = std::max(max_y, point.y());
            }
        }
    }

    if (!has_value) {
        axis->setRange(0.0, 1.0);
        return;
    }

    const double span = max_y - min_y;
    const double padding = span > 0.0 ? span * 0.1 : std::max(std::abs(min_y) * 0.1, 1.0);
    axis->setRange(min_y - padding, max_y + padding);
}
}

SimulationWindow::SimulationWindow(QWidget *parent) : QMainWindow(parent), engine_(std::make_unique<Boeing747Physics>()) {
    setupLayouts();
    
    timer_ = std::make_unique<QTimer>(this);
    connect(timer_.get(), &QTimer::timeout, this, &SimulationWindow::updateSimulationTick);
    timer_->start(100); 
}

void SimulationWindow::setupLayouts() {
    QPalette palette = QApplication::palette();
    palette.setColor(QPalette::Window, QColor(16, 19, 24));
    palette.setColor(QPalette::WindowText, QColor(242, 242, 242));
    palette.setColor(QPalette::Base, QColor(32, 36, 43));
    palette.setColor(QPalette::AlternateBase, QColor(41, 49, 59));
    palette.setColor(QPalette::Text, Qt::white);
    palette.setColor(QPalette::Button, QColor(41, 49, 59));
    palette.setColor(QPalette::ButtonText, Qt::white);
    palette.setColor(QPalette::Highlight, QColor(49, 93, 120));
    palette.setColor(QPalette::HighlightedText, Qt::white);
    QApplication::setPalette(palette);

    auto central_widget = std::make_unique<QWidget>();
    central_widget->setAutoFillBackground(true);
    auto main_layout = std::make_unique<QVBoxLayout>();

    // Top Half
    auto top_layout = std::make_unique<QHBoxLayout>();
    
    auto stream_box = std::make_unique<QWidget>();
    stream_box->setAutoFillBackground(true);
    QPalette stream_palette = stream_box->palette();
    stream_palette.setColor(QPalette::Window, Qt::black);
    stream_box->setPalette(stream_palette);
    auto stream_layout = std::make_unique<QVBoxLayout>();
    auto stream_widget = std::make_unique<StreamlineWidget>();
    streamline_view_ = stream_widget.get();
    stream_layout->addWidget(stream_widget.release());
    stream_box->setLayout(stream_layout.release());
    top_layout->addWidget(stream_box.release(), 2);

    auto graph_layout = std::make_unique<QGridLayout>();
    createGraphs(graph_layout.get());
    top_layout->addLayout(graph_layout.release(), 1);
    
    main_layout->addLayout(top_layout.release(), 1);

    // Bottom Half
    auto bottom_layout = std::make_unique<QGridLayout>();
    createAutopilot(bottom_layout.get());
    createSliders(bottom_layout.get());
    createRadios(bottom_layout.get());
    createReadouts(bottom_layout.get());

    main_layout->addLayout(bottom_layout.release(), 0);

    central_widget->setLayout(main_layout.release());
    setCentralWidget(central_widget.release());
}

void SimulationWindow::createGraphs(QGridLayout* layout) {
    auto createChart = [](const QString& title, QPointer<QLineSeries>& series,
                          QPointer<QValueAxis>& axisX, QPointer<QValueAxis>& axisY,
                          QPointer<TelemetryChartView>& chart_view) {
        auto chart = std::make_unique<QChart>();
        auto local_series = std::make_unique<QLineSeries>();
        series = local_series.get();
        local_series->setColor(QColor(80, 205, 255));
        chart->addSeries(local_series.release());
        chart->setTitle(title);
        chart->setTitleBrush(Qt::white);
        
        auto x_axis = std::make_unique<QValueAxis>();
        axisX = x_axis.get();
        axisX->setRange(0, 10);
        axisX->setLabelsColor(Qt::white);
        axisX->setLinePenColor(QColor(145, 155, 170));
        axisX->setGridLineColor(QColor(65, 75, 88));
        chart->addAxis(x_axis.release(), Qt::AlignBottom);
        series->attachAxis(axisX);
        
        auto y_axis = std::make_unique<QValueAxis>();
        axisY = y_axis.get();
        axisY->setLabelsColor(Qt::white);
        axisY->setLinePenColor(QColor(145, 155, 170));
        axisY->setGridLineColor(QColor(65, 75, 88));
        chart->addAxis(y_axis.release(), Qt::AlignLeft);
        series->attachAxis(axisY);

        chart->legend()->hide();
        chart->setBackgroundBrush(QBrush(QColor(16, 19, 24)));
        chart->setPlotAreaBackgroundVisible(true);
        chart->setPlotAreaBackgroundBrush(QBrush(QColor(10, 13, 17)));

        auto view = std::make_unique<TelemetryChartView>(chart.release());
        chart_view = view.get();
        return view;
    };

    layout->addWidget(createChart("Structure Load Factor Graph", load_factor_s_, axisX_lf,
                                  axisY_lf, load_factor_chart_).release(), 0, 0);
    layout->addWidget(createChart("Specific Excess Power Graph", excess_power_s_, axisX_ep,
                                  axisY_ep, excess_power_chart_).release(), 0, 1);
    layout->addWidget(createChart("Wing Root Bending Moment Graph", bending_moment_s_, axisX_bm,
                                  axisY_bm, bending_moment_chart_).release(), 1, 0);
    
    auto drag_chart = std::make_unique<QChart>();
    auto s1 = std::make_unique<QLineSeries>(); drag_ind_s_ = s1.get(); drag_ind_s_->setName("Induced");
    auto s2 = std::make_unique<QLineSeries>(); drag_par_s_ = s2.get(); drag_par_s_->setName("Parasitic");
    auto s3 = std::make_unique<QLineSeries>(); drag_tot_s_ = s3.get(); drag_tot_s_->setName("Total");
    
    drag_chart->addSeries(s1.release());
    drag_chart->addSeries(s2.release());
    drag_chart->addSeries(s3.release());
    drag_chart->setTitle("Drag Breakdown");
    drag_ind_s_->setColor(QColor(65, 190, 255));
    drag_par_s_->setColor(QColor(255, 190, 70));
    drag_tot_s_->setColor(QColor(110, 230, 140));

    auto x_axis = std::make_unique<QValueAxis>();
    axisX_drag = x_axis.get();
    axisX_drag->setRange(0, 10);
    axisX_drag->setLabelsColor(Qt::white);
    axisX_drag->setLinePenColor(QColor(145, 155, 170));
    axisX_drag->setGridLineColor(QColor(65, 75, 88));
    drag_chart->addAxis(x_axis.release(), Qt::AlignBottom);
    
    auto y_axis = std::make_unique<QValueAxis>();
    axisY_drag = y_axis.get();
    axisY_drag->setLabelsColor(Qt::white);
    axisY_drag->setLinePenColor(QColor(145, 155, 170));
    axisY_drag->setGridLineColor(QColor(65, 75, 88));
    drag_chart->addAxis(y_axis.release(), Qt::AlignLeft);

    for(auto series : drag_chart->series()) {
        series->attachAxis(axisX_drag);
        series->attachAxis(axisY_drag);
    }
    
    drag_chart->setTitleBrush(Qt::white);
    drag_chart->setBackgroundBrush(QBrush(QColor(16, 19, 24)));
    drag_chart->setPlotAreaBackgroundVisible(true);
    drag_chart->setPlotAreaBackgroundBrush(QBrush(QColor(10, 13, 17)));
    auto drag_view = std::make_unique<TelemetryChartView>(drag_chart.release());
    drag_chart_view_ = drag_view.get();
    layout->addWidget(drag_view.release(), 1, 1);
}

void SimulationWindow::createAutopilot(QGridLayout* layout) {
    auto ap_layout = std::make_unique<QVBoxLayout>();
    ap_layout->addWidget(new QLabel("Autopilot\nModes"));

    btn_alt_hold = new QPushButton("ALT\nHOLD");
    btn_alt_hold->setCheckable(true);
    edit_alt_hold = new QLineEdit("10000");
    ap_layout->addWidget(btn_alt_hold);
    ap_layout->addWidget(edit_alt_hold);

    btn_mach_hold = new QPushButton("MACH\nHOLD");
    btn_mach_hold->setCheckable(true);
    edit_mach_hold = new QLineEdit("0.85");
    ap_layout->addWidget(btn_mach_hold);
    ap_layout->addWidget(edit_mach_hold);

    btn_hdg_sel = new QPushButton("HDG\nSEL");
    btn_hdg_sel->setCheckable(true);
    edit_hdg_sel = new QLineEdit("0");
    ap_layout->addWidget(btn_hdg_sel);
    ap_layout->addWidget(edit_hdg_sel);

    layout->addLayout(ap_layout.release(), 0, 0);
}

void SimulationWindow::createSliders(QGridLayout* layout) {
    auto slider_container = std::make_unique<QVBoxLayout>();
    auto vertical_sliders = std::make_unique<QHBoxLayout>();

    auto createVCol = [](const QString& title, int min, int max, int start, const QStringList& labels, QPointer<QSlider>& out_slider) {
        auto col = std::make_unique<QVBoxLayout>();
        col->addWidget(new QLabel(title), 0, Qt::AlignHCenter);
        
        auto sl_layout = std::make_unique<QHBoxLayout>();
        auto slider = std::make_unique<QSlider>(Qt::Vertical);
        slider->setRange(min, max);
        slider->setTickPosition(QSlider::TicksRight);
        slider->setValue(start);
        slider->setInvertedAppearance(true); 
        out_slider = slider.get();
        sl_layout->addWidget(slider.release());

        auto lbl_layout = std::make_unique<QVBoxLayout>();
        for (const auto& lbl : labels) {
            lbl_layout->addWidget(new QLabel(lbl));
        }
        sl_layout->addLayout(lbl_layout.release());
        col->addLayout(sl_layout.release());
        return col;
    };

    vertical_sliders->addLayout(createVCol("Flaps", 0, 6, 0, {"UP", "1", "5", "10", "20", "25", "30"}, flaps_slider_).release());
    vertical_sliders->addLayout(createVCol("Pitch", -15, 20, 0, {"+20", "0", "-15"}, pitch_slider_).release());
    vertical_sliders->addLayout(createVCol("Speed Brake", 0, 3, 0, {"Retracted", "Armed", "Flight Detent", "UP"}, speed_brake_slider_).release());
    vertical_sliders->addLayout(createVCol("Throttle", 0, 6, 2, {"TO/GA", "MCT", "CLB", "Cont. Cruise", "IDLE", "REV IDLE", "REV MAX"}, throttle_slider_).release());

    slider_container->addLayout(vertical_sliders.release());

    auto roll_layout = std::make_unique<QVBoxLayout>();
    roll_slider_ = new QSlider(Qt::Horizontal);
    roll_slider_->setRange(-45, 45);
    roll_slider_->setValue(0);
    roll_layout->addWidget(roll_slider_);
    roll_layout->addWidget(new QLabel("Roll"), 0, Qt::AlignHCenter);
    slider_container->addLayout(roll_layout.release());

    layout->addLayout(slider_container.release(), 0, 1);
}

void SimulationWindow::createRadios(QGridLayout* layout) {
    auto radio_layout = std::make_unique<QVBoxLayout>();
    
    alt_group_ = new QButtonGroup(this);
    radio_layout->addWidget(new QLabel("Altitude:"));
    QRadioButton* r1 = new QRadioButton("Sea Level"); r1->setChecked(true);
    QRadioButton* r2 = new QRadioButton("3048m/10000ft");
    QRadioButton* r3 = new QRadioButton("10668m/35000ft");
    alt_group_->addButton(r1, 0); alt_group_->addButton(r2, 1); alt_group_->addButton(r3, 2);
    radio_layout->addWidget(r1); radio_layout->addWidget(r2); radio_layout->addWidget(r3);

    spd_group_ = new QButtonGroup(this);
    radio_layout->addWidget(new QLabel("Airspeed/Mach:"));
    QRadioButton* s1 = new QRadioButton("150 KIAS");
    QRadioButton* s2 = new QRadioButton("250 KIAS"); s2->setChecked(true);
    QRadioButton* s3 = new QRadioButton("0.85 Mach");
    spd_group_->addButton(s1, 0); spd_group_->addButton(s2, 1); spd_group_->addButton(s3, 2);
    radio_layout->addWidget(s1); radio_layout->addWidget(s2); radio_layout->addWidget(s3);

    wt_group_ = new QButtonGroup(this);
    radio_layout->addWidget(new QLabel("Aircraft Weight:"));
    QRadioButton* w1 = new QRadioButton("MTOW");
    QRadioButton* w2 = new QRadioButton("Typical Cruise"); w2->setChecked(true);
    QRadioButton* w3 = new QRadioButton("MLW");
    wt_group_->addButton(w1, 0); wt_group_->addButton(w2, 1); wt_group_->addButton(w3, 2);
    radio_layout->addWidget(w1); radio_layout->addWidget(w2); radio_layout->addWidget(w3);

    layout->addLayout(radio_layout.release(), 0, 2);
}

void SimulationWindow::createReadouts(QGridLayout* layout) {
    auto readout_layout = std::make_unique<QVBoxLayout>();
    auto addReadout = [&](QPointer<QLabel>& lbl, const QString& title) {
        lbl = new QLabel(title);
        lbl->setAutoFillBackground(true);
        QPalette readout_palette = lbl->palette();
        readout_palette.setColor(QPalette::Window, QColor(41, 49, 59));
        readout_palette.setColor(QPalette::WindowText, Qt::white);
        lbl->setPalette(readout_palette);
        readout_layout->addWidget(lbl);
    };

    addReadout(readout_dyn_press_, "Dyn. Pressure:\n");
    addReadout(readout_lift_, "Lift:\n");
    addReadout(readout_thrust_, "Thrust:\n");
    addReadout(readout_ff_, "Fuel Flow Rate:\n");
    addReadout(readout_tsfc_, "TSFC:\n");
    addReadout(readout_ld_, "Lift/Drag:\n");

    layout->addLayout(readout_layout.release(), 0, 3);
}

void SimulationWindow::updateSimulationTick() {
    if (!engine_ || !flaps_slider_ || !speed_brake_slider_ || !throttle_slider_ || !pitch_slider_ ||
        !roll_slider_ || !alt_group_ || !spd_group_ || !wt_group_ || !streamline_view_ ||
        !load_factor_s_ || !excess_power_s_ || !bending_moment_s_ || !drag_ind_s_ || !drag_par_s_ ||
        !drag_tot_s_ || !axisX_lf || !axisX_ep || !axisX_bm || !axisX_drag || !axisY_lf ||
        !axisY_ep || !axisY_bm || !axisY_drag || !load_factor_chart_ || !excess_power_chart_ ||
        !bending_moment_chart_ || !drag_chart_view_ || !readout_dyn_press_ ||
        !readout_lift_ || !readout_thrust_ || !readout_ff_ || !readout_tsfc_ || !readout_ld_) {
        return;
    }

    // Sliders to enum mapping.
    const std::array<Flaps, 7> flap_mapping = {Flaps::UP, Flaps::F1, Flaps::F5, Flaps::F10, Flaps::F20, Flaps::F25, Flaps::F30};
    const std::array<SpeedBrake, 4> sb_mapping = {SpeedBrake::RETRACTED, SpeedBrake::ARMED, SpeedBrake::FLIGHT_DETENT, SpeedBrake::UP};
    const std::array<Throttle, 7> th_mapping = {Throttle::TOGA, Throttle::MCT, Throttle::CLB, Throttle::CONT_CRUISE, Throttle::IDLE, Throttle::REV_IDLE, Throttle::REV_MAX};

    const int flap_index = std::clamp(flaps_slider_->value(), 0, static_cast<int>(flap_mapping.size()) - 1);
    const int sb_index = std::clamp(speed_brake_slider_->value(), 0, static_cast<int>(sb_mapping.size()) - 1);
    const int throttle_index = std::clamp(throttle_slider_->value(), 0, static_cast<int>(th_mapping.size()) - 1);

    engine_->setFlaps(flap_mapping[static_cast<size_t>(flap_index)]);
    engine_->setSpeedBrake(sb_mapping[static_cast<size_t>(sb_index)]);
    engine_->setThrottle(th_mapping[static_cast<size_t>(throttle_index)]);

    // Reverse inversion for Pitch.
    engine_->setPitch(static_cast<double>(pitch_slider_->value() * -1));
    engine_->setRoll(static_cast<double>(roll_slider_->value()));

    // Radios: default to the current checked selection if a group is unset.
    engine_->setAltitudeType(static_cast<AltitudeLevel>(safeCheckedButtonValue(alt_group_, 0)));
    engine_->setAirspeedType(static_cast<AirspeedTarget>(safeCheckedButtonValue(spd_group_, 1)));
    engine_->setWeightType(static_cast<AircraftWeight>(safeCheckedButtonValue(wt_group_, 1)));

    // AP state
    AutopilotState ap;
    ap.alt_hold = btn_alt_hold && btn_alt_hold->isChecked();
    ap.h_target = parseDoubleOrDefault(edit_alt_hold, 10000.0);
    ap.mach_hold = btn_mach_hold && btn_mach_hold->isChecked();
    ap.m_target = parseDoubleOrDefault(edit_mach_hold, 0.85);
    ap.hdg_sel = btn_hdg_sel && btn_hdg_sel->isChecked();
    ap.rad_target = parseDoubleOrDefault(edit_hdg_sel, 0.0);
    engine_->updateAutopilot(ap);

    SimTelemetry tel = engine_->tick(0.1);

    static bool stall_alert_active = false;
    if (tel.is_stalled && !stall_alert_active) {
        QApplication::beep();
        stall_alert_active = true;
    } else if (!tel.is_stalled) {
        stall_alert_active = false;
    }

    readout_dyn_press_->setText(QString("Dyn. Pressure:\n%1 Pa").arg(tel.dyn_pressure, 0, 'f', 2));
    readout_lift_->setText(QString("Lift:\n%1 N").arg(tel.lift, 0, 'f', 2));
    readout_thrust_->setText(QString("Thrust:\n%1 N").arg(tel.thrust, 0, 'f', 2));
    readout_ff_->setText(QString("Fuel Flow Rate:\n%1 kg/s").arg(tel.fuel_flow, 0, 'f', 4));
    readout_tsfc_->setText(QString("TSFC:\n%1").arg(tel.tsfc, 0, 'f', 6));
    readout_ld_->setText(QString("Lift/Drag:\n%1").arg(tel.lift_drag_ratio, 0, 'f', 2));

    // Update charts and the time axis without forcing the vertical axis to zero.
    const double visible_min_x = std::max(0.0, tel.time_sec - 10.0);
    auto updateSeries = [tel, visible_min_x](QPointer<QLineSeries> s, double val,
                                             QPointer<QValueAxis> axX, QPointer<QValueAxis> axY) {
        if (!s || !axX || !axY || !std::isfinite(tel.time_sec) || !std::isfinite(val)) {
            return;
        }

        s->append(tel.time_sec, val);
        const auto points = s->points();
        int points_to_remove = 0;
        while (points_to_remove < points.size() && points.at(points_to_remove).x() < visible_min_x) {
            ++points_to_remove;
        }
        if (points_to_remove > 0) {
            s->removePoints(0, points_to_remove);
        }
        if (tel.time_sec > 10.0) {
            axX->setRange(tel.time_sec - 10.0, tel.time_sec);
        }
        updateVerticalAxis(axY, {s}, visible_min_x, tel.time_sec);
    };

    updateSeries(load_factor_s_, tel.load_factor, axisX_lf, axisY_lf);
    updateSeries(excess_power_s_, tel.spec_excess_power, axisX_ep, axisY_ep);
    updateSeries(bending_moment_s_, tel.root_bending_moment, axisX_bm, axisY_bm);
    load_factor_chart_->setValueText(QString("Current: %1 g").arg(tel.load_factor, 0, 'f', 3));
    excess_power_chart_->setValueText(QString("Current: %1 m/s").arg(tel.spec_excess_power, 0, 'f', 2));
    bending_moment_chart_->setValueText(QString("Current: %1 MN m").arg(
        tel.root_bending_moment / 1.0e6, 0, 'f', 2));

    if (drag_ind_s_ && drag_par_s_ && drag_tot_s_) {
        const double drag_values[] = {tel.drag_induced, tel.drag_parasite, tel.drag_total};
        if (std::isfinite(tel.time_sec) &&
            std::all_of(std::begin(drag_values), std::end(drag_values),
                        [](double value) { return std::isfinite(value); })) {
            drag_ind_s_->append(tel.time_sec, tel.drag_induced);
            drag_par_s_->append(tel.time_sec, tel.drag_parasite);
            drag_tot_s_->append(tel.time_sec, tel.drag_total);
            for (QLineSeries* series : {drag_ind_s_.data(), drag_par_s_.data(), drag_tot_s_.data()}) {
                const auto points = series->points();
                int points_to_remove = 0;
                while (points_to_remove < points.size() && points.at(points_to_remove).x() < visible_min_x) {
                    ++points_to_remove;
                }
                if (points_to_remove > 0) {
                    series->removePoints(0, points_to_remove);
                }
            }
            updateVerticalAxis(axisY_drag, {drag_ind_s_, drag_par_s_, drag_tot_s_},
                               visible_min_x, tel.time_sec);
            drag_chart_view_->setValueText(QString("I:%1 P:%2 T:%3 kN")
                .arg(tel.drag_induced / 1000.0, 0, 'f', 0)
                .arg(tel.drag_parasite / 1000.0, 0, 'f', 0)
                .arg(tel.drag_total / 1000.0, 0, 'f', 0));
        }
    }
    if (tel.time_sec > 10.0 && axisX_drag) {
        axisX_drag->setRange(tel.time_sec - 10.0, tel.time_sec);
    }

    streamline_view_->updateData(engine_->generateRenderData());
}