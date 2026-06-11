#ifndef IROB_TRAJEC_MAKER_PANEL_HPP
#define IROB_TRAJEC_MAKER_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

// iRob command message
#include "irob_msgs/msg/irob_cmd_msg.hpp"

#include <QString>
#include <QByteArray>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>

namespace irob_trajec_maker_panel
{
	class TrajectoryPanel : public rviz_common::Panel{
		Q_OBJECT
	
		public:
			explicit TrajectoryPanel(QWidget * parent = 0);
			~TrajectoryPanel() override;
			
			void onInitialize() override;
			
		protected:
			// Layout : Main box
			QVBoxLayout*	qBoxMainLayout_;	
			
			// Layout : Header
			QVBoxLayout*	qBoxHeader_;
			QLabel* 		qHeaderLabel_;
			QLabel*			qOpenFileName_;
			
			// Layout : Robot Status and Logo
			QHBoxLayout*	qBoxRobotStatusLogo_;
			QVBoxLayout*	qBoxRobotStatus_;
			QVBoxLayout*	qBoxActualRobotStatus_;
			QHBoxLayout*	qBoxRobotLogo_;
			QPixmap*		qLogoPixmap_;
			QLabel*			qRobotClubIcon_;
			QLabel*			qLabelPoseNumber_;
			QLabel*			qLabelPositionX_;
			QLabel*			qLabelPositionY_;
			QLabel*			qLabelPosePoints_;
			QLabel*			qLabelRobotState_;
			
			// Layout : Control buttons
			QVBoxLayout*	qBoxControlBtns_;
			QPushButton*	qButtonUndo_;
			QPushButton*	qButtonPublishPath_;
			QPushButton*	qButtonEmergency_;
			QPushButton*	qButtonSavePath_;
			QPushButton*	qButtonOpenPath_;
			
			QFileDialog*	qFileSavePathCSV_;
			QString*		qStrSavePathName_;
			
			QFileDialog*	qFileOpenPathCSV_;
			QString*		qStrOpenPathName_;
			
			// ROS node transfer
			std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
			
			rclcpp::Node::SharedPtr	nRvizNode;
			
			// Communication between python node and rviz panel node
			rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr	pubToPyNode;
			
			rclcpp::Subscription<irob_msgs::msg::IrobCmdMsg>::SharedPtr	subFromPyNode;
			
			irob_msgs::msg::IrobCmdMsg toBackendMsg;
			
			void vIrobBackendCallbackHandler(const irob_msgs::msg::IrobCmdMsg::SharedPtr backendMsg);
		
			// Wall timer for connection status check of backend to panel 
			rclcpp::TimerBase::SharedPtr timer_;
			void vIrobBackendAliveCheck();
			
			bool IsConnected;
		
		private Q_SLOTS:
		
			// Button callback
			void vButtonUndoTrajectory();
			void vButtonPublishPath();
			void vButtonSaveFilePicker();
			void vButtonOpenFilePicker();
			
	};
	
}  

#endif  