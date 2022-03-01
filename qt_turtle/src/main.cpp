#include <rclcpp/rclcpp.hpp>
#include <qt_turtle/ui/mainwindow.h>
#include <QApplication>
#include <signal.h>

std::unique_ptr<QApplication> a = nullptr;
std::unique_ptr<MainWindow> w = nullptr;

void sig_handler(int signo) {
    switch (signo) {
    case SIGINT:    // Ctrl + C
    case SIGTERM:   // killall
        w->close();
        break;
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    rclcpp::init(argc, argv);

    a = std::make_unique<QApplication>(argc, argv);
    w = std::make_unique<MainWindow>();

    w->show();
    return a->exec();
}
