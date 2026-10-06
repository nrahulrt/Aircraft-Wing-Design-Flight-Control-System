#ifndef SIMULATION_UI_H
#define SIMULATION_UI_H

#include <QMainWindow>
#include <QTimer>
#include <QPainter>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QLineEdit>
#include <QPointer>
#include <QPolygonF>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QResizeEvent>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>
#include <memory>
#include <utility>
#include "PhysicsEngine.h"

class TelemetryChartView : public QChartView {
public:
    explicit TelemetryChartView(QChart* chart, QWidget* parent = nullptr)
        : QChartView(chart, parent) {
        setRenderHint(QPainter::Antialiasing);
        value_label_ = new QLabel(this);
        value_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        value_label_->setAutoFillBackground(true);
        QPalette label_palette = value_label_->palette();
        label_palette.setColor(QPalette::Window, QColor(0, 0, 0, 210));
        label_palette.setColor(QPalette::WindowText, Qt::white);
        value_label_->setPalette(label_palette);
    }

    void setValueText(QString text) {
        value_label_->setText(std::move(text));
        const int label_width = std::min(width() - 16,
            value_label_->fontMetrics().horizontalAdvance(value_label_->text()) + 12);
        value_label_->setGeometry(8, height() - 30, std::max(0, label_width), 22);
    }

protected:
    void resizeEvent(QResizeEvent* event) override {
        QChartView::resizeEvent(event);
        const int label_width = std::min(width() - 16,
            value_label_->fontMetrics().horizontalAdvance(value_label_->text()) + 12);
        value_label_->setGeometry(8, height() - 30, std::max(0, label_width), 22);
    }
private:
    QLabel* value_label_ = nullptr;
};

class StreamlineWidget : public QWidget {
    Q_OBJECT
public:
    explicit StreamlineWidget(QWidget *parent = nullptr) : QWidget(parent) {
        auto animation_timer = new QTimer(this);
        connect(animation_timer, &QTimer::timeout, this, [this]() {
            animation_phase_ = std::fmod(animation_phase_ + 0.025, 1.0);
            update();
        });
        animation_timer->start(40);
    }
    void updateData(std::shared_ptr<const RenderData> render_data) {
        render_data_ = std::move(render_data);
        update(); 
    }
protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), Qt::black);

        if (!render_data_) return;

        const double scale = std::max(1.0, std::min((width() - 24.0) / 16.0,
                                                    (height() - 24.0) / 9.5));
        auto mapPoint = [this, scale](const Point2D& pt) -> QPointF {
            return QPointF(width() / 2.0 + pt.x * scale, height() / 2.0 - pt.y * scale);
        };
        auto drawAnimatedArrow = [&painter, &mapPoint](const std::vector<Point2D>& line,
                                                       double progress, bool reverse,
                                                       const QColor& color) {
            if (line.size() < 2) {
                return;
            }

            const size_t index = 1 + static_cast<size_t>(
                progress * static_cast<double>(line.size() - 2));
            const QPointF tip = mapPoint(line[index]);
            const QPointF previous = mapPoint(line[reverse ? index + 1 : index - 1]);
            const double dx = tip.x() - previous.x();
            const double dy = tip.y() - previous.y();
            const double length = std::hypot(dx, dy);
            if (length <= 0.0) {
                return;
            }

            const QPointF direction(dx / length, dy / length);
            const QPointF normal(-direction.y(), direction.x());
            const QPointF base = tip - direction * 8.0;
            painter.setPen(QPen(color, 1.7));
            painter.drawLine(tip, base + normal * 3.5);
            painter.drawLine(tip, base - normal * 3.5);
        };

        // Draw streamlines and moving flow markers first.
        painter.setPen(QPen(QColor(150, 190, 205), 1.2));
        painter.setBrush(Qt::NoBrush);
        size_t line_number = 0;
        for (const auto& line : render_data_->streamlines) {
            if (line.size() < 2) continue;

            QPainterPath path;
            path.moveTo(mapPoint(line.front()));
            for (size_t i = 1; i < line.size(); ++i) {
                path.lineTo(mapPoint(line[i]));
            }
            painter.drawPath(path);

            const double progress = std::fmod(animation_phase_ + line_number * 0.13, 1.0);
            drawAnimatedArrow(line, progress, false, QColor(225, 240, 245));
            ++line_number;
        }

        for (const auto& zone : render_data_->recirculation_zones) {
            if (zone.size() < 2) continue;
            QPainterPath path;
            path.moveTo(mapPoint(zone.front()));
            for (size_t i = 1; i < zone.size(); ++i) {
                path.lineTo(mapPoint(zone[i]));
            }
            painter.setPen(QPen(QColor(255, 155, 80), 1.5, Qt::DashLine));
            painter.drawPath(path);
            const double progress = std::fmod(animation_phase_ * 0.7, 1.0);
            drawAnimatedArrow(zone, progress, true, QColor(255, 195, 110));
        }

        for (const Point2D& separation : render_data_->separation_points) {
            const QPointF point = mapPoint(separation);
            painter.setPen(QPen(QColor(255, 90, 70), 1.5));
            painter.setBrush(QColor(255, 90, 70));
            painter.drawEllipse(point, 4.0, 4.0);
            painter.setPen(QColor(255, 135, 115));
            painter.drawText(point + QPointF(6.0, -5.0), "SEP");
        }
        if (!render_data_->separation_points.empty()) {
            painter.setPen(QColor(225, 225, 225));
            painter.drawText(rect().adjusted(8, 8, -8, -8),
                             Qt::AlignRight | Qt::AlignBottom,
                             "Red: estimated separation  |  dashed: recirculation");
        }

        // Mask a 10-pixel band around the section so flow remains visibly clear
        // of the airfoil at every widget scale.
        QPolygonF airfoilPolygon;
        if (!render_data_->airfoil_top.empty()) {
            for (const auto& pt : render_data_->airfoil_top) {
                airfoilPolygon.append(mapPoint(pt));
            }
            for (auto it = render_data_->airfoil_bottom.rbegin(); it != render_data_->airfoil_bottom.rend(); ++it) {
                airfoilPolygon.append(mapPoint(*it));
            }
            QPainterPath airfoilPath;
            airfoilPath.addPolygon(airfoilPolygon);
            airfoilPath.closeSubpath();

            QPainterPathStroker clearance_stroker;
            clearance_stroker.setWidth(22.0);
            clearance_stroker.setJoinStyle(Qt::RoundJoin);
            const QPainterPath clearance_halo = clearance_stroker.createStroke(airfoilPath);
            painter.setPen(Qt::NoPen);
            painter.setBrush(Qt::black);
            painter.drawPath(clearance_halo);

            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(150, 155, 162));
            painter.drawPolygon(airfoilPolygon, Qt::OddEvenFill);
            painter.setBrush(Qt::NoBrush);
            painter.setPen(QPen(QColor(245, 245, 245), 2.0));
            painter.drawPolygon(airfoilPolygon);
        }
    }
private:
    std::shared_ptr<const RenderData> render_data_;
    double animation_phase_ = 0.0;
};

class SimulationWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit SimulationWindow(QWidget *parent = nullptr);
    ~SimulationWindow() override = default;

private slots:
    void updateSimulationTick();

private:
    void setupLayouts();
    void createGraphs(QGridLayout* layout);
    void createAutopilot(QGridLayout* layout);
    void createSliders(QGridLayout* layout);
    void createRadios(QGridLayout* layout);
    void createReadouts(QGridLayout* layout);

    std::unique_ptr<Boeing747Physics> engine_;
    std::unique_ptr<QTimer> timer_;
    
    QPointer<StreamlineWidget> streamline_view_;

    QPointer<QLineSeries> load_factor_s_;
    QPointer<QLineSeries> excess_power_s_;
    QPointer<QLineSeries> bending_moment_s_;
    QPointer<QLineSeries> drag_ind_s_;
    QPointer<QLineSeries> drag_par_s_;
    QPointer<QLineSeries> drag_tot_s_;
    QPointer<QValueAxis> axisX_lf, axisX_ep, axisX_bm, axisX_drag;
    QPointer<QValueAxis> axisY_lf, axisY_ep, axisY_bm, axisY_drag;
    QPointer<TelemetryChartView> load_factor_chart_;
    QPointer<TelemetryChartView> excess_power_chart_;
    QPointer<TelemetryChartView> bending_moment_chart_;
    QPointer<TelemetryChartView> drag_chart_view_;

    QPointer<QPushButton> btn_alt_hold;
    QPointer<QLineEdit> edit_alt_hold;
    QPointer<QPushButton> btn_mach_hold;
    QPointer<QLineEdit> edit_mach_hold;
    QPointer<QPushButton> btn_hdg_sel;
    QPointer<QLineEdit> edit_hdg_sel;

    QPointer<QSlider> flaps_slider_;
    QPointer<QSlider> pitch_slider_;
    QPointer<QSlider> speed_brake_slider_;
    QPointer<QSlider> throttle_slider_;
    QPointer<QSlider> roll_slider_;
    
    QPointer<QButtonGroup> alt_group_;
    QPointer<QButtonGroup> spd_group_;
    QPointer<QButtonGroup> wt_group_;

    QPointer<QLabel> readout_dyn_press_;
    QPointer<QLabel> readout_lift_;
    QPointer<QLabel> readout_thrust_;
    QPointer<QLabel> readout_ff_;
    QPointer<QLabel> readout_tsfc_;
    QPointer<QLabel> readout_ld_;
};

#endif