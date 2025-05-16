#include "GUI.h"

#include <iostream>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QKeyEvent>
#include <QApplication>
#include <QFont>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

GUI::GUI(std::shared_ptr<rclcpp::Node> node, QWidget *parent) 
    : QWidget(parent), node_(node), estopActive(false), spacePressed(false), isHumanTurn(true)
{
    // Create the publisher for e-stop status
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/estop", 10);

    // publisher for the Dead Man's Switch
    dms_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/dms", 10);

    // publisher for the turn switch
    turn_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/turn", 10);
        
    // Set up the UI
    setupUI();
    
    // Set focus policy to enable key events
    setFocusPolicy(Qt::StrongFocus);
    
    // Install event filter on application to catch key events globally
    QApplication::instance()->installEventFilter(this);
}

bool GUI::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Space && !keyEvent->isAutoRepeat() && !spacePressed) {
            // Only process initial key press, ignore auto-repeats
            spacePressed = true;
            updateMasterControlStatus(true);
            return true; // Event handled
        }
    } else if (event->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Space && !keyEvent->isAutoRepeat() && spacePressed) {
            // Only process actual key release, not auto-repeat releases
            spacePressed = false;
            updateMasterControlStatus(false);
            return true; // Event handled
        }
    }
    // Standard event processing
    return QObject::eventFilter(obj, event);
}

void GUI::updateMasterControlStatus(bool active)
{
    if (active)  {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        dms_pub_->publish(message);

        masterControlBar->setStyleSheet("background-color: #4CAF50; border-radius: 5px;"); // Green
        masterControlLabel->setText("Robot Movement Master Control: ACTIVE");

    } else {
        auto message = std_msgs::msg::Bool();
        message.data = false;
        dms_pub_->publish(message);


        masterControlBar->setStyleSheet("background-color: #F44336; border-radius: 5px;"); // Red
        masterControlLabel->setText("Robot Movement Master Control: INACTIVE (Hold Space)");
    }

    // Update status message
    updateStatus();
}

void GUI::toggleTurn()
{
    isHumanTurn = !isHumanTurn;
    
    if (isHumanTurn) {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        turn_pub_->publish(message);

        turnButton->setText("Human Turn");
        turnIndicator->setStyleSheet("background-color: #4CAF50; border: 2px solid #2E7D32; border-radius: 20px; opacity: 0.8;"); // Green with faded border
    } else {
        auto message = std_msgs::msg::Bool();
        message.data = false;
        turn_pub_->publish(message);

        turnButton->setText("Robot Turn");
        turnIndicator->setStyleSheet("background-color: #F44336; border: 2px solid #B71C1C; border-radius: 20px; opacity: 0.8;"); // Red with faded border
    }

    // Update status message
    updateStatus();
}

void GUI::toggleEStop()
{
    estopActive = !estopActive;
    
    if (estopActive) {
        estopIndicator->setStyleSheet("background-color: #F44336; border-radius: 20px;"); // Red
        estopLabel->setText("ESTOP ON");

        // Disable and gray out turn button
        turnButton->setEnabled(false);
        turnButton->setStyleSheet("background-color: grey; color: white; font-weight: bold; font-size: 16px; border-radius: 10px;");

        if (!isHumanTurn) {
            toggleTurn();
        }
    } else {
        estopIndicator->setStyleSheet("background-color: #4CAF50; border-radius: 20px;"); // Green
        estopLabel->setText("ESTOP OFF");

        // Re-enable turn button and restore style
        turnButton->setEnabled(true);
        turnButton->setStyleSheet("font-weight: bold; font-size: 16px;");
    }
    
    // Publish E-Stop status
    auto message = std_msgs::msg::Bool();
    message.data = estopActive;
    estop_pub_->publish(message);

    // Update status message
    updateStatus();
}

void GUI::updateStatus()
{
    if (estopActive) {
        statusLabel->setText("Emergency Stop Activated! Press Estop again to reset");
    } else if (!isHumanTurn && !spacePressed) {
        statusLabel->setText("Hold Dead Man's Switch to enable movement");
    } else if (!isHumanTurn && spacePressed) {
        statusLabel->setText("CAUTION - robot is currently executing its turn");
    } else {
        statusLabel->setText("Ready - Human turn");
    }
}

void GUI::setupUI()
{
    // Main layout
    QVBoxLayout *mainLayout = new QVBoxLayout();
    
    // Title
    QLabel *titleLabel = new QLabel("UR3 Robot Control Interface");
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(titleLabel);
    
    // Master Control Bar
    QHBoxLayout *masterLayout = new QHBoxLayout();
    masterControlBar = new QFrame();
    masterControlBar->setMinimumHeight(40);
    masterControlBar->setStyleSheet("background-color: #F44336; border-radius: 5px;"); // Red
    masterControlLabel = new QLabel("Robot Movement Master Control: INACTIVE (Hold Space)");
    masterControlLabel->setAlignment(Qt::AlignCenter);
    masterControlLabel->setStyleSheet("color: white; font-weight: bold;");
    
    QHBoxLayout *masterBarLayout = new QHBoxLayout(masterControlBar);
    masterBarLayout->addWidget(masterControlLabel);
    
    mainLayout->addWidget(masterControlBar);
    mainLayout->addSpacing(10);
    
    // Control Buttons Row
    QHBoxLayout *controlButtonsLayout = new QHBoxLayout();
    
    // E-Stop Button
    estopButton = new QPushButton("Emergency Stop");
    estopButton->setMinimumHeight(60);
    //estopButton->setStyleSheet("background-color: #F44336; color: white; font-weight: bold; font-size: 16px; border-radius: 10px;");
    estopButton->setStyleSheet("font-weight: bold; font-size: 16px;");
    
    // E-Stop Indicator
    estopIndicator = new QFrame();
    estopIndicator->setFixedSize(120, 40);
    estopIndicator->setStyleSheet("background-color: #4CAF50; border-radius: 20px;"); // Green initially
    
    QHBoxLayout *estopIndicatorLayout = new QHBoxLayout(estopIndicator);
    estopLabel = new QLabel("ESTOP OFF");
    estopLabel->setAlignment(Qt::AlignCenter);
    estopLabel->setStyleSheet("color: white; font-weight: bold;");
    estopIndicatorLayout->addWidget(estopLabel);
    
    // Add E-Stop controls to their container
    QVBoxLayout *estopContainerLayout = new QVBoxLayout();
    estopContainerLayout->addWidget(estopButton);
    estopContainerLayout->addWidget(estopIndicator, 0, Qt::AlignCenter);
    
    // Human/Robot Turn Button
    turnButton = new QPushButton("Human Turn");
    turnButton->setMinimumHeight(60);
    turnButton->setStyleSheet("font-weight: bold; font-size: 16px;");
    
    // Turn Indicator
    turnIndicator = new QFrame();
    turnIndicator->setFixedSize(120, 40);
    turnIndicator->setStyleSheet("background-color: #4CAF50; border: 2px solid #2E7D32; border-radius: 20px; opacity: 0.8;"); // Green with faded border initially
    
    // Add Turn controls to their container
    QVBoxLayout *turnContainerLayout = new QVBoxLayout();
    turnContainerLayout->addWidget(turnButton);
    turnContainerLayout->addWidget(turnIndicator, 0, Qt::AlignCenter);
    
    // Add both control sets to the row
    controlButtonsLayout->addLayout(estopContainerLayout);
    controlButtonsLayout->addLayout(turnContainerLayout);
    
    mainLayout->addLayout(controlButtonsLayout);
    mainLayout->addSpacing(15);
    
    // Status Label
    statusLabel = new QLabel("Ready");
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setStyleSheet("font-weight: bold; margin-top: 10px;");
    mainLayout->addWidget(statusLabel);
    
    // Set the main layout
    setLayout(mainLayout);
    
    // Connect button signal
    connect(estopButton, &QPushButton::clicked, this, &GUI::toggleEStop);
    connect(turnButton, &QPushButton::clicked, this, &GUI::toggleTurn);
    
    // Set window properties
    setWindowTitle("UR3 Robot Control");
    setMinimumSize(600, 400);
}

#ifndef GUI_H
#define GUI_H

#include <QWidget>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>


// Forward declarations
class QSlider;
class QLabel;
class QPushButton;
class QFrame;
class QKeyEvent;
class QEvent;

class GUI : public QWidget
{
    Q_OBJECT

public:
    GUI(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);

protected:
    // Event filter to handle application-wide events
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void toggleEStop();
    void toggleTurn();
    
private:
    void setupUI();
    void updateMasterControlStatus(bool active);
    void updateStatus();
    
    // ROS2 node and publisher
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> estop_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> dms_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> turn_pub_;
    

    QLabel *statusLabel;
    
    // E-Stop components
    QPushButton *estopButton;
    QFrame *estopIndicator;
    QLabel *estopLabel;
    bool estopActive;
    
    // Master Control components
    QFrame *masterControlBar;
    QLabel *masterControlLabel;
    bool spacePressed;  // To track spacebar state
    
    // Turn control components
    QPushButton *turnButton;
    QFrame *turnIndicator;
    bool isHumanTurn;
};

#endif // GUI_H