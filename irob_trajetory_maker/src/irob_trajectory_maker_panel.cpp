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
			qHeaderLabel_	= new QLabel("Robot Club KMITL : iRob Trajectory Maker");
			
			qHeaderLabel_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qBoxHeader_->addWidget(qHeaderLabel_);
		}
		
		// Layout : Robot Status and Logo
		{
			qBoxRobotStatusLogo_	= new QHBoxLayout;
			qBoxRobotStatus_		= new QVBoxLayout;
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
			qLabelPositionX_	= new QLabel("X: 0.0");
			qLabelPositionY_	= new QLabel("Y: 0.0");
			qLabelRobotState_	= new QLabel("Status: OK");
			
			qLabelPositionX_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qLabelPositionY_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			qLabelRobotState_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
			
			qBoxRobotStatus_->addWidget(qLabelPositionX_);
			qBoxRobotStatus_->addWidget(qLabelPositionY_);
			qBoxRobotStatus_->addWidget(qLabelRobotState_);
			
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
			
			qBoxControlBtns_->addWidget(qButtonUndo_);
			qBoxControlBtns_->addWidget(qButtonPublishPath_);
			qBoxControlBtns_->addWidget(qButtonSavePath_);
			
			// Connect buttons to callback
			QObject::connect(
				qButtonSavePath_, 
				&QPushButton::released, 
				this, 
				&TrajectoryPanel::vButtonFilePicker
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
		
		
	}
	
	
	void TrajectoryPanel::vButtonFilePicker(){
		RCLCPP_INFO(
			nRvizNode->get_logger(),
			"Saving Path file..."
		);
		
		// Create file dialog
		qFileSavePathCSV_	= new QFileDialog;
		
		qFileSavePathCSV_->setWindowTitle("Save Path CSV");
		qFileSavePathCSV_->setAcceptMode(QFileDialog::AcceptSave);
		qFileSavePathCSV_->setNameFilter("iRob Path file (*.csv *.CSV)");
		if(qFileSavePathCSV_->exec()){
			qStrSavePathName_ = new QString(qFileSavePathCSV_->selectedFiles()[0]);
			RCLCPP_INFO(
				nRvizNode->get_logger(),
				"Saved as %s", qStrSavePathName_->toUtf8().data()
			);
		}
	}
}  

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(irob_trajec_maker_panel::TrajectoryPanel, rviz_common::Panel)