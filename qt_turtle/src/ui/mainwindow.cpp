#include <qt_turtle/ui/mainwindow.h>
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_node = std::make_shared<RosNode>();

    m_ros_thread = std::make_unique<std::thread>(
        [this]{
            rclcpp::spin(m_node);
        }
    );

    m_update_timer = std::make_unique<QTimer>(this);

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(m_update_timer.get(), &QTimer::timeout, this, &MainWindow::update_timer_tick);

    m_update_timer->start((int)(1000.0/60.0));
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    if(m_ros_thread->joinable()){
        m_ros_thread->join();
    }

    delete ui;
}

void MainWindow::update_timer_tick(void){
    ui->txb_x->setText(QString::number(m_node->x(), 'f', 4));
    ui->txb_y->setText(QString::number(m_node->y(), 'f', 4));
    ui->txb_th->setText(QString::number(m_node->theta(), 'f', 4));
}
