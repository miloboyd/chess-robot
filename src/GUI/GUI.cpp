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
    // Create the publisher for joint positions
    joint_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "ur3/joint_positions", 10);
        
    // Create the publisher for e-stop status
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/estop", 10);

    dms_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/dms", 10);

    turn_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/turn", 10);
        
    // Set up the UI
    setupUI();
    
    // Connect signals and slots
    connectSignals();
    
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
}

void GUI::toggleEStop()
{
    estopActive = !estopActive;
    
    if (estopActive) {
        estopIndicator->setStyleSheet("background-color: #F44336; border-radius: 20px;"); // Red
        estopLabel->setText("ESTOP ON");
    } else {
        estopIndicator->setStyleSheet("background-color: #4CAF50; border-radius: 20px;"); // Green
        estopLabel->setText("ESTOP OFF");
    }
    
    // Publish E-Stop status
    auto message = std_msgs::msg::Bool();
    message.data = estopActive;
    estop_pub_->publish(message);
    
    // For E-Stop, also update status label
    statusLabel->setText(estopActive ? "Emergency Stop Activated" : "Ready");
}

void GUI::sendJointPositions()
{
    // Update position labels first
    updateJointLabel(1, joint1Slider->value());
    updateJointLabel(2, joint2Slider->value());
    updateJointLabel(3, joint3Slider->value());
    updateJointLabel(4, joint4Slider->value());
    updateJointLabel(5, joint5Slider->value());
    updateJointLabel(6, joint6Slider->value());
    
    auto message = std_msgs::msg::Float64MultiArray();
    
    // Get values from sliders
    message.data = {
        static_cast<double>(joint1Slider->value()) / 100.0,
        static_cast<double>(joint2Slider->value()) / 100.0,
        static_cast<double>(joint3Slider->value()) / 100.0,
        static_cast<double>(joint4Slider->value()) / 100.0,
        static_cast<double>(joint5Slider->value()) / 100.0,
        static_cast<double>(joint6Slider->value()) / 100.0
    };
    
    // Publish the message
    joint_pub_->publish(message);
    
    // Log the sent values
    std::cout << "Sent joint positions: ";
    for (auto val : message.data) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

void GUI::homePosition()
{
    // Set all sliders to home position (0)
    joint1Slider->setValue(0);
    joint2Slider->setValue(0);
    joint3Slider->setValue(0);
    joint4Slider->setValue(0);
    joint5Slider->setValue(0);
    joint6Slider->setValue(0);
    
    // Send the joint positions
    sendJointPositions();
}

void GUI::updateJointLabel(int joint, int value)
{
    double angle = static_cast<double>(value) / 100.0;
    
    switch(joint) {
        case 1:
            joint1Label->setText(QString("Joint 1: %1 rad").arg(angle));
            break;
        case 2:
            joint2Label->setText(QString("Joint 2: %1 rad").arg(angle));
            break;
        case 3:
            joint3Label->setText(QString("Joint 3: %1 rad").arg(angle));
            break;
        case 4:
            joint4Label->setText(QString("Joint 4: %1 rad").arg(angle));
            break;
        case 5:
            joint5Label->setText(QString("Joint 5: %1 rad").arg(angle));
            break;
        case 6:
            joint6Label->setText(QString("Joint 6: %1 rad").arg(angle));
            break;
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
    
    // E-Stop Button (Larger)
    estopButton = new QPushButton("Emergency Stop");
    estopButton->setMinimumHeight(60);
    estopButton->setStyleSheet("background-color: #F44336; color: white; font-weight: bold; font-size: 16px; border-radius: 10px;");
    
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
    turnButton->setStyleSheet("font-weight: bold; font-size: 14px;");
    
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
    
    // Joint Control Group
    QGroupBox *jointGroupBox = new QGroupBox("Joint Control");
    QVBoxLayout *jointLayout = new QVBoxLayout();
    
    // Create sliders and labels for each joint
    createJointControl(jointLayout, 1);
    createJointControl(jointLayout, 2);
    createJointControl(jointLayout, 3);
    createJointControl(jointLayout, 4);
    createJointControl(jointLayout, 5);
    createJointControl(jointLayout, 6);
    
    jointGroupBox->setLayout(jointLayout);
    mainLayout->addWidget(jointGroupBox);
    
    // Button Group
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    
    // Home Button
    QPushButton *homeButton = new QPushButton("Home Position");
    homeButton->setMinimumHeight(40);
    buttonLayout->addWidget(homeButton);
    
    // Send Button
    QPushButton *sendButton = new QPushButton("Send Joint Positions");
    sendButton->setMinimumHeight(40);
    sendButton->setStyleSheet("font-weight: bold;");
    buttonLayout->addWidget(sendButton);
    
    mainLayout->addLayout(buttonLayout);
    
    // Status Label
    statusLabel = new QLabel("Ready");
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setStyleSheet("font-weight: bold; margin-top: 10px;");
    mainLayout->addWidget(statusLabel);
    
    // Set the main layout
    setLayout(mainLayout);
    
    // Connect button signals
    connect(homeButton, &QPushButton::clicked, this, &GUI::homePosition);
    connect(sendButton, &QPushButton::clicked, this, &GUI::sendJointPositions);
    connect(estopButton, &QPushButton::clicked, this, &GUI::toggleEStop);
    connect(turnButton, &QPushButton::clicked, this, &GUI::toggleTurn);
    
    // Set window properties
    setWindowTitle("UR3 Robot Control");
    setMinimumSize(600, 700);
}

void GUI::createJointControl(QVBoxLayout *layout, int jointNum)
{
    QHBoxLayout *controlLayout = new QHBoxLayout();
    
    // Create label
    QLabel *label = new QLabel(QString("Joint %1: 0.00 rad").arg(jointNum));
    
    // Create slider
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(-314, 314);  // Range from -3.14 to 3.14, multiplied by 100
    slider->setValue(0);          // Default to 0
    
    // Store references to the sliders and labels
    switch(jointNum) {
        case 1:
            joint1Slider = slider;
            joint1Label = label;
            break;
        case 2:
            joint2Slider = slider;
            joint2Label = label;
            break;
        case 3:
            joint3Slider = slider;
            joint3Label = label;
            break;
        case 4:
            joint4Slider = slider;
            joint4Label = label;
            break;
        case 5:
            joint5Slider = slider;
            joint5Label = label;
            break;
        case 6:
            joint6Slider = slider;
            joint6Label = label;
            break;
    }
    
    // Add widgets to layout
    controlLayout->addWidget(label);
    controlLayout->addWidget(slider);
    
    layout->addLayout(controlLayout);
}

void GUI::connectSignals()
{
    // Connect slider value changes to label updates
    connect(joint1Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(1, value);
    });
    
    connect(joint2Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(2, value);
    });
    
    connect(joint3Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(3, value);
    });
    
    connect(joint4Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(4, value);
    });
    
    connect(joint5Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(5, value);
    });
    
    connect(joint6Slider, &QSlider::valueChanged, [this](int value) {
        this->updateJointLabel(6, value);
    });
}