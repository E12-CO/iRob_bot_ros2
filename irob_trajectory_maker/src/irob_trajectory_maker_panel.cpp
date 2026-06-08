// iRob trajectory maker rViz2 panel plug in.
// By TinLethax at Robot Club KMITL (RB26)
#include <irob_trajectory_maker/irob_trajectory_maker_panel.hpp>


namespace irob_trajec_maker_panel
{
	TrajectoryPanel::TrajectoryPanel(QWidget* parent) : Panel(parent){
		// Layout : create Main box
		{
			qBoxMainLayout_		= new QVBoxLayout(this);
		}
		
		// Layout : Header
		{
			qBoxHeader_ 	= new QVBoxLayout(this);
			qHeaderLabel_	= new QLabel("Robot Club KMITL  iRob Trajectory Maker");
			qOpenFileName_ = new QLabel("Current Path File :");
			
			qHeaderLabel_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qOpenFileName_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			
			qBoxHeader_->addWidget(qHeaderLabel_);
			qBoxHeader_->addWidget(qOpenFileName_);
		}
		
		// Layout : Robot Status and Logo
		{
			qBoxRobotStatusLogo_	= new QHBoxLayout;
			qBoxRobotStatus_		= new QVBoxLayout;
			qBoxActualRobotStatus_	= new QVBoxLayout;
			qBoxRobotLogo_			= new QHBoxLayout;
			
			// Club's logo
			qRobotClubIcon_ = new QLabel("");
			qLogoPixmap_	= new QPixmap(rviz_common::loadPixmap("package://irob_trajectory_maker/icons/classes/RobotClub.jpeg"));
			qRobotClubIcon_->setPixmap(
				qLogoPixmap_->scaled(
					64, 64, 
					Qt::IgnoreAspectRatio, 
					Qt::SmoothTransformation
				)
			);
			qRobotClubIcon_->setScaledContents(false);// Don't scale the logo when expanding the rViz panel
			
			// Robot status
			qLabelPoseNumber_	= new QLabel("Pose Number: 0");
			//qLabelPositionX_	= new QLabel("X: 0.0");
			//qLabelPositionY_	= new QLabel("Y: 0.0");
			qLabelPosePoints_	= new QLabel("Total Pose points: 0");
			qLabelRobotState_	= new QLabel("Status: OK");
			
			qLabelPoseNumber_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			//qLabelPositionX_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			//qLabelPositionY_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qLabelPosePoints_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qLabelRobotState_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			
			qBoxActualRobotStatus_->addWidget(qLabelPoseNumber_);
			//qBoxActualRobotStatus_->addWidget(qLabelPositionX_);
			//qBoxActualRobotStatus_->addWidget(qLabelPositionY_);
			qBoxActualRobotStatus_->addWidget(qLabelPosePoints_);
			qBoxActualRobotStatus_->addWidget(qLabelRobotState_);
			
			qBoxRobotStatus_->addLayout(qBoxActualRobotStatus_);
			
			qBoxRobotLogo_->addWidget(qRobotClubIcon_, 0, Qt::AlignRight);
			
			qBoxRobotStatusLogo_->addLayout(qBoxRobotStatus_);
			qBoxRobotStatusLogo_->addLayout(qBoxRobotLogo_);
		}
		
		// Layout : Control Buttons
		{
			qBoxControlBtns_	= new QVBoxLayout;
			
			qButtonUndo_ 		= new QPushButton("Undo pose");
			qButtonPublishPath_	= new QPushButton("Publish Path");
			qButtonSavePath_	= new QPushButton("Save Path to file");	
			qButtonOpenPath_	= new QPushButton("Open Path from file");
			
			qBoxControlBtns_->addWidget(qButtonUndo_);
			qBoxControlBtns_->addWidget(qButtonPublishPath_);
			qBoxControlBtns_->addWidget(qButtonSavePath_);
			qBoxControlBtns_->addWidget(qButtonOpenPath_);
			
			// Connect buttons to callback
			QObject::connect(
				qButtonUndo_,
				&QPushButton::released,
				this,
				&TrajectoryPanel::vButtonUndoTrajectory
				);
			
			QObject::connect(
				qButtonPublishPath_,
				&QPushButton::released,
				this,
				&TrajectoryPanel::vButtonPublishPath
				);
			
			QObject::connect(
				qButtonSavePath_, 
				&QPushButton::released, 
				this, 
				&TrajectoryPanel::vButtonSaveFilePicker
				);
				
			QObject::connect(
				qButtonOpenPath_, 
				&QPushButton::released, 
				this, 
				&TrajectoryPanel::vButtonOpenFilePicker
				);	
		}
		
		// Layout : add everything to main box
		{
			qBoxMainLayout_->addLayout(qBoxHeader_);
			qBoxMainLayout_->addLayout(qBoxRobotStatusLogo_);
			qBoxMainLayout_->addLayout(qBoxControlBtns_);
		}
		
	}

	TrajectoryPanel::~TrajectoryPanel() = default;
	
	void TrajectoryPanel::onInitialize(){
		node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
		
		if(node_ptr_ == nullptr){
			RCLCPP_ERROR(
				rclcpp::get_logger("TrajectoryPanel"),
				"Can't get Node instant from the RViz, make sure it's running!"
			);
			return;
		}
		
		nRvizNode = node_ptr_->get_raw_node();
		
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Starting iRob Trajectory Maker panel"
			);
		
		// Setting up communication with the python backend node
		pubToPyNode = nRvizNode->create_publisher<irob_msgs::msg::IrobCmdMsg>("/irob_panel_to_backend", 10);
		
		subFromPyNode = nRvizNode->create_subscription<irob_msgs::msg::IrobCmdMsg>(
				"/irob_backend_to_panel",
				10,
				std::bind(
					&TrajectoryPanel::vIrobBackendCallbackHandler,
					this,
					std::placeholders::_1)
			);
		
	}
	
	void TrajectoryPanel::vIrobBackendCallbackHandler(const irob_msgs::msg::IrobCmdMsg::SharedPtr backendMsg){
		std::string backendCmd, backendData;
		std::string updateStr;
		std::size_t splitPos;
		
		if (splitPos = backendMsg->irobcmd.find(','), splitPos != std::string::npos) {
			backendCmd = backendMsg->irobcmd.substr(0, 4);
			backendData = backendMsg->irobcmd.substr(splitPos+1);
		}else{
			backendCmd = backendMsg->irobcmd;
		}
		
		if (backendCmd == "pose"){
			updateStr = "Pose Number: ";
			updateStr += backendData;
			qLabelPoseNumber_->setText(QString(updateStr.c_str()));
		}else if(backendCmd == "pnts"){
			updateStr = "Total Pose points: ";
			updateStr += backendData;
			qLabelPosePoints_->setText(QString(updateStr.c_str()));
		}else if(backendCmd == "ping"){
			
			
		}
		
	}
	
	void TrajectoryPanel::vButtonUndoTrajectory(){
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Undo the latest pose..."
		);
		
		toBackendMsg.irobcmd = "undo";
		pubToPyNode->publish(toBackendMsg);
	}
	
	void TrajectoryPanel::vButtonPublishPath(){
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Publishing path..."
		);
		
		toBackendMsg.irobcmd = "pub";
		pubToPyNode->publish(toBackendMsg);
	}
	
	void TrajectoryPanel::vButtonSaveFilePicker(){
		std::string openFileNameStr;
		
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Saving Path file..."
		);
		
		// Create file dialog
		qFileSavePathCSV_	= new QFileDialog;
		
		qFileSavePathCSV_->setWindowTitle("Save Path to CSV file");
		qFileSavePathCSV_->setAcceptMode(QFileDialog::AcceptSave);
		qFileSavePathCSV_->setNameFilter("iRob Path file (*.csv *.CSV)");
		if(qFileSavePathCSV_->exec()){
			qStrSavePathName_ = new QString(qFileSavePathCSV_->selectedFiles()[0]);
			
			openFileNameStr = qStrSavePathName_->toStdString();
			openFileNameStr = "Current Path File : " + openFileNameStr.substr(openFileNameStr.find_last_of("/") + 1);
			
			qOpenFileName_->setText(QString(openFileNameStr.c_str()));
			
			RCLCPP_INFO(
				nRvizNode->get_logger(),
				"Saved as %s", qStrSavePathName_->toUtf8().data()
			);
			
			// Send save command to python backend node
			toBackendMsg.irobcmd = "save,";
			toBackendMsg.irobcmd += qStrSavePathName_->toStdString();
			
			pubToPyNode->publish(toBackendMsg);
		}
	}
	
	void TrajectoryPanel::vButtonOpenFilePicker(){
		std::string openFileNameStr;
		
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Openning Path file..."
		);
			
		qFileOpenPathCSV_	= new QFileDialog;
		
		qFileOpenPathCSV_->setWindowTitle("Open Path CSV file");
		qFileOpenPathCSV_->setAcceptMode(QFileDialog::AcceptOpen);
		qFileOpenPathCSV_->setNameFilter("iRob Path file (*.csv *.CSV)");
		if(qFileOpenPathCSV_->exec()){
			qStrOpenPathName_ = new QString(qFileOpenPathCSV_->selectedFiles()[0]);
			
			openFileNameStr = qStrOpenPathName_->toStdString();
			openFileNameStr = "Current Path File : " + openFileNameStr.substr(openFileNameStr.find_last_of("/") + 1);
			
			qOpenFileName_->setText(QString(openFileNameStr.c_str()));
			
			RCLCPP_INFO(
				nRvizNode->get_logger(),
				"Open file %s ", 
				qStrOpenPathName_->toUtf8().data()
			);
			
			toBackendMsg.irobcmd = "open,";
			toBackendMsg.irobcmd += qStrOpenPathName_->toStdString();
			
			pubToPyNode->publish(toBackendMsg);
		}
		
	}
	
	
}  

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(irob_trajec_maker_panel::TrajectoryPanel, rviz_common::Panel)