#ifndef IROB_TRAJEC_MAKER_PANEL_HPP
#define IROB_TRAJEC_MAKER_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

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
			
			// Layout : Robot Status ad Logo
			QHBoxLayout*	qBoxRobotStatusLogo_;
			QVBoxLayout*	qBoxRobotStatus_;
			QHBoxLayout*	qBoxRobotLogo_;
			QPixmap*		qLogoPixmap_;
			QLabel*			qRobotClubIcon_;
			QLabel*			qLabelPositionX_;
			QLabel*			qLabelPositionY_;
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
		
		private Q_SLOTS:
			void vButtonFilePicker();
	};
	
}  

#endif  