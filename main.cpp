#include <QApplication>
#include "SimulationUI.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    std::unique_ptr<SimulationWindow> window = std::make_unique<SimulationWindow>();
    window->resize(1400, 900);
    window->setWindowTitle("747 Aerodynamic & Systems Simulation");
    window->show();

    return app.exec();
}