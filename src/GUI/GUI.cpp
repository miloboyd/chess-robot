#include "GUI.h"

GUI::GUI(std::shared_ptr<rclcpp::Node> node, QWidget *parent) 
    : QWidget(parent), node_(node), estopActive(false), spacePressed(false), isRobotTurn(false), difficulty_(0), started_(false)
{
    // Create the publisher for e-stop status
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/estop", 10);

    // publisher for the Dead Man's Switch
    dms_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "ur3/dms", 10);

    // publisher for the turn switch
    turn_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/move_complete", 10);

    turn_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/move_complete", 10,
        std::bind(&GUI::turn_callback, this, std::placeholders::_1));

    // publisher for the turn switch
    diff_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "ur3/diff", 10);

    //initialise the start service
    start_service_client_ = node_->create_client<std_srvs::srv::SetBool>("ur3/start_signal");

    dms_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GUI::publishDMSState, this)
    );
        
    // Set up the UI
    setupUI();
    
    // Set focus policy to enable key events
    setFocusPolicy(Qt::StrongFocus);
    
    // Install event filter on application to catch key events globally
    QApplication::instance()->installEventFilter(this);

    RCLCPP_INFO(node_->get_logger(), "GUI initialized.");
}

void GUI::publishDMSState()
{
    if (started_) {
        auto message = std_msgs::msg::Bool();
        message.data = spacePressed;
        dms_pub_->publish(message);
    }
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

        masterControlBar->setStyleSheet("background-color: #4CAF50; border-radius: 5px;"); // Green
        masterControlLabel->setText("Robot Movement Master Control: ACTIVE");

    } else {


        masterControlBar->setStyleSheet("background-color: #F44336; border-radius: 5px;"); // Red
        masterControlLabel->setText("Robot Movement Master Control: INACTIVE (Hold Space)");
    }

    // Update status message
    updateStatus();
}

void GUI::turn_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == false) {
        toggleTurn();
    }
}

void GUI::toggleTurn()
{
    if (started_) {
        isRobotTurn = !isRobotTurn;

        if (!isRobotTurn) {
            //auto message = std_msgs::msg::Bool();
            //message.data = true;
            //turn_pub_->publish(message);  
            if (!estopActive) {turnButton->setEnabled(true);}
                
            turnButton->setText("Human Turn");
            turnIndicator->setStyleSheet("background-color: #4CAF50; border: 2px solid #2E7D32; border-radius: 20px; opacity: 0.8;"); // Green with faded border
        } else {
            auto message = std_msgs::msg::Bool();
            message.data = true;
            turn_pub_->publish(message);

            turnButton->setEnabled(false);
            turnButton->setText("Robot Turn");
            turnIndicator->setStyleSheet("background-color: #F44336; border: 2px solid #B71C1C; border-radius: 20px; opacity: 0.8;"); // Red with faded border
        }
    }

    

    // Update status message
    updateStatus();
}

void GUI::toggleEStop()
{
    if (started_) {
        estopActive = !estopActive;
        
        if (estopActive) {
            estopIndicator->setStyleSheet("background-color: #F44336; border-radius: 20px;"); // Red
            estopLabel->setText("ESTOP ON");

            // Disable and gray out turn button
            turnButton->setEnabled(false);
            //turnButton->setStyleSheet("background-color: grey; color: white; font-weight: bold; font-size: 16px; border-radius: 10px;");

            // if (!isRobotTurn) {
            //     toggleTurn();
            // }
        } else {
            estopIndicator->setStyleSheet("background-color: #4CAF50; border-radius: 20px;"); // Green
            estopLabel->setText("ESTOP OFF");

            // Re-enable turn button and restore style
            if (!isRobotTurn) {
                turnButton->setEnabled(true);
                turnButton->setStyleSheet("font-weight: bold; font-size: 16px;");
            }
        }
        
        // Publish E-Stop status
        auto message = std_msgs::msg::Bool();
        message.data = estopActive;
        estop_pub_->publish(message);

        // Update status message
        updateStatus();
    }
}

void GUI::updateStatus()
{
    if (estopActive) {
        statusLabel->setText("Emergency Stop Activated! Press Estop again to reset");
    } else if (isRobotTurn && !spacePressed) {
        statusLabel->setText("Hold Dead Man's Switch to enable movement");
    } else if (isRobotTurn && spacePressed) {
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
    masterControlBar->setMinimumWidth(400);
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
    
    mainLayout->addSpacing(60);  // White space above the new elements

    // Difficulty slider and label
    QVBoxLayout *difficultyLayout = new QVBoxLayout();
    difficultySlider = new QSlider(Qt::Horizontal);
    difficultySlider->setRange(1, 20);
    difficultySlider->setTickInterval(1);
    difficultySlider->setTickPosition(QSlider::TicksBelow);
    difficultySlider->setSingleStep(1);
    difficultySlider->setPageStep(1);
    
    difficultyLabel = new QLabel("Difficulty: 1");
    difficultyLabel->setAlignment(Qt::AlignCenter);

    difficultyLayout->addWidget(difficultySlider);
    difficultyLayout->addWidget(difficultyLabel);

    // Start button
    startButton = new QPushButton("START");
    startButton->setMinimumHeight(50);
    startButton->setStyleSheet("font-weight: bold; font-size: 16px;");

    // Bottom layout combining slider and button
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->addStretch(1);

    // Add your difficulty slider and label
    bottomLayout->addLayout(difficultyLayout);

    // Add spacing between slider and button (optional)
    bottomLayout->addSpacing(100);

    // Add the START button
    bottomLayout->addWidget(startButton);

    // Add stretchable space to the right
    bottomLayout->addStretch(1);
    
    mainLayout->addLayout(bottomLayout);

    // Set the main layout
    setLayout(mainLayout);
    
    // Connect button signal
    connect(estopButton, &QPushButton::clicked, this, &GUI::toggleEStop);
    connect(turnButton, &QPushButton::clicked, this, &GUI::toggleTurn);
    
    connect(difficultySlider, &QSlider::valueChanged, this, [this](int value) {
        difficultyLabel->setText(QString("Difficulty: %1").arg(value));
        difficulty_ = value;
    });

    connect(startButton, &QPushButton::clicked, this, [this]() {

        if (!start_service_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "Start service not recieved.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;  // Send "start" signal

        start_service_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
                if (response.get()->success) {
                    RCLCPP_INFO(node_->get_logger(), "Start acknowledged: %s", response.get()->message.c_str());

                    //send difficulty over publisher
                    auto message = std_msgs::msg::String();
                    message.data = std::to_string(difficulty_);
                    diff_pub_->publish(message);

                    started_ = true;
                    startButton->setEnabled(false);
                    difficultySlider->setEnabled(false);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Start service call failed: %s", response.get()->message.c_str());
                }
            });
    });
    
    // Set window properties
    setWindowTitle("UR3 Robot Control");
    setMinimumSize(400, 600);
}
