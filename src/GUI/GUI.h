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
    void sendJointPositions();
    void homePosition();
    void updateJointLabel(int joint, int value);
    void toggleEStop();
    void toggleTurn();
    
private:
    void setupUI();
    void createJointControl(QVBoxLayout *layout, int jointNum);
    void connectSignals();
    void updateMasterControlStatus(bool active);
    
    // ROS2 node and publisher
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> joint_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> estop_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> dms_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> turn_pub_;
    
    // UI Components
    QSlider *joint1Slider, *joint2Slider, *joint3Slider, *joint4Slider, *joint5Slider, *joint6Slider;
    QLabel *joint1Label, *joint2Label, *joint3Label, *joint4Label, *joint5Label, *joint6Label;
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