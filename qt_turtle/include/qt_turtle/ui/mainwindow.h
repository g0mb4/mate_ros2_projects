#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <qt_turtle/ros/ros_node.hpp>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow* ui;

    std::shared_ptr<RosNode> m_node { nullptr };
    std::unique_ptr<std::thread> m_ros_thread { nullptr };
    std::unique_ptr<QTimer> m_update_timer { nullptr };
    void update_timer_tick(void);
};
#endif // MAINWINDOW_H
