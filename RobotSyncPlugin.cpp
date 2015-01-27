/**
 * @author Yuki Suga (SSR) ysuga@ysuga.net
 * 
 * Choreonoid���̃��f���̃f�[�^��RobotSync�Ƃ������O��RTC��DoubleSeq
 * 
 * �܂��C���f���̖��O��robot�łȂ���΂Ȃ�Ȃ��D
 * ����́Ccnoid�t�@�C�����e�L�X�g�G�f�B�^�Œ��ڕύX����΁C�ύX�ł���D
 *
 * Start�����RobotSync*.rtc�Ƃ���RT�R���|�[�l���g���쐬
 * Connect����ƁCRTC������A�N�e�B�u�����C�V�~�����[�^���̃��f���̊Ԑڊp�x���o�͂���D
 *
 * RTC�ɂ́Cdefault_mask��default_angle_radian�Ƃ����R���t�B�O���[�V����������C
 * ������CSV�`���̕�����ŁConActivated�Ő��l�z��Ƃ��ĉ��߂����D
 * default_mask, default_angle_radian�̗�����CSV�œn�����v�f�̐��́C���{�b�g���f���̊֐߂̐��Ɠ����łȂ���΂Ȃ�Ȃ��D
 * ���Ƃ��΁C�֐ߐ���3�̃��f���ł���΁Cmask��1,1,0�ŁCangle_radian��1.0, 1.58, 0.9�������Ƃ���D
 * �����C�֐ߐ���3�łȂ���΁C����RTC��Connect���悤�Ƃ���ƃG���[�ƂȂ�D
 * �v�f����������Active�������D
 *
 * default_mask�́Cdefault_angle_radian�Ŏw�肳���f�t�H���g�p�x�l�̂ݏo�͂���}�X�N�֐߂��C�ۂ������߂�t���O�ŁC
 * 1���w�肳�ꂽ�֐߂́Cdefault_angle_radian�̊p�x�݂̂��o�͂���D
 * 0���w�肳���ƁCchoreonoid��̃��f���̊p�x���o�͂���D
 * 
 */


#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/JointPath>
#include <cnoid/ToolBar>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include <cnoid/Timer>
#include <cnoid/TimeBar>

#include <boost/thread/thread.hpp>

#include "RobotSync.h"


#include <math.h>
using namespace boost;
using namespace cnoid;


// ��������f�o�b�O�֘A�̃}�N���̒�`
#define putln(msg)  MessageView::instance()->putln((msg))
#define puts(msg) MessageView::instance()->put((msg))

#define TAG "[RobotSyncPlugin]"
#define error(msg) do{puts(TAG " ERROR    :");putln(msg);}while(false)

#ifdef ROBOT_VERBOSE
#define verbose(msg) do{puts(TAG " VERBOSE  :");putln(msg);}while(false)
#define verbose3(msg, x, y, z) do{\
		std::ostringstream oss;\
		oss << x << ", " << y << ", " << z;\
		puts(TAG " VERBOSE  :");puts(msg);putln(oss.str());}while(false)
#else
#define verbose(msg)
#define verbose3(msg, x, y, z)
#endif

#define info(msg) do{puts(TAG " INFO     :");putln(msg);}while(false)
#define trace(msg) do{puts(TAG " TRACE    :");putln(msg);}while(false)
#define debug(msg) do{puts(TAG " DEBUG    :");putln(msg);}while(false)
#define debug3(msg, x, y, z) do{\
		std::ostringstream oss;\
		oss << x << ", " << y << ", " << z;\
		puts(TAG " DEBUG    :");puts(msg);putln(oss.str());}while(false)

#define print_exc(msg) do{puts(TAG " EXCEPTION:");putln(msg);}while(false)
// �f�o�b�O�֘A�}�N����`�͂����܂�


// �v���O�C���̏��
enum ROBOT_CONTROLLER_MODE {
	MODE_START, // RTC���N����
	MODE_STOP, // RTC����~��
	MODE_CONNECTED, // RTC�ƃV�~�����[�^���̃��f�����ڑ�����Ă���C�����I�ɖڕW�p�x��f���o�����
};


void rtcManagerMainLoop() {
        RTC::Manager::instance().runManager();
}

class RobotSyncPlugin : public Plugin
{
public:
	RTC::Manager* manager ;

	RobotSyncPlugin() : Plugin("RobotSync")
	{
		require("Body");
	}


	/**
	 * �����ŏ�����
	 */
	virtual bool initialize()
	{
		// Robot�Ƃ������O�̃c�[���o�[�����D
		ToolBar* bar = new ToolBar("Robot");
		m_pStartButton = bar->addButton("Start/Stop"); // Start/Stop�{�^����ǉ�
		m_pStartButton->sigClicked().connect(bind(&RobotSyncPlugin::onStartButtonClicked, this));
		m_pConnectButton = bar->addButton("Connect/Disconnect"); // Connect/Disconnect�{�^����ǉ�
		m_pConnectButton->sigClicked().connect(bind(&RobotSyncPlugin::onConnectButtonClicked, this));
		addToolBar(bar);

		// �^�C�}�[���쐬�D�������s�̏���
		m_Timer.sigTimeout().connect(bind(&RobotSyncPlugin::onTimer, this));

		info("RobotSyncPlugin is successfully initialized.");	

		// ��~��Ԃɂ��Ă���
		m_Mode = MODE_STOP;

		// RT�R���|�[�l���g�̋N�������D���̎��_�Ńl�[���T�[�r�X���N�����Ă��Ȃ��ƃA�E�g�D
		const char* argv[] = {
            "choreonoid",
            "-o", "logger.enable: NO",
            "-o", "manager.shutdown_on_nortcs: NO",
            "-o", "manager.shutdown_auto: NO",
            "-o", "naming.formats: %n.rtc",
#ifdef Q_OS_WIN32
            // To reduce the startup time on Windows
            "-o", "corba.args: -ORBclientCallTimeOutPeriod 100"
#endif
            //"-o", "corba.nameservers: localhost",
            //"-o", "exec_cxt.periodic.type: SynchExtTriggerEC",
            //"-o", "exec_cxt.periodic.rate: 1000000",
            //"-o", "manager.is_master: YES"
            //"-o", "logger.enable: YES",
            //"-o", "logger.file_name: stdout",
            //"-o", "logger.log_level: TRACE",
            //"-o", "corba.args: -ORBendPoint giop:tcp::2809 -ORBpoaUniquePersistentSystemIds 1"
        };

#ifdef Q_OS_WIN32
        int numArgs = 11;
#else
        int numArgs = 9;
#endif
		// Manager�̋N��
        manager = RTC::Manager::init(numArgs, const_cast<char**>(argv));
        manager->activateManager();
            
#ifdef Q_OS_WIN32
        omniORB::setClientCallTimeout(0); // reset the global timeout setting?
#endif
		// Manager��RT�R���|�[�l���g��o�^
		RobotSync::registerFactory(manager, "RobotSync");

		// Manager�̎��s�͕ʃX���b�h������s�������̂ŃX���b�h�𗧂Ă�D
		rtcManagerMainLoopThread = boost::thread(rtcManagerMainLoop);


		return true;
	}

	// Start�{�^���������ꂽ��
	void onStartButtonClicked() {
		trace("Start Button Clicked.");
		if(isStarted()) { // Start���Ă���Stop
			if(isConnected()) { // ����Connected�Ȃ�Stop�ł��Ȃ��D
				info("RobotSync Plugin is already connected to robot model. Can not stop.");
			} else {
				stop();
			}
		} else { // Stop���Ă���Start
			start();
		}
		if(isStarted()) { // Start�ł��Ă���^�C�}�N��
			startTimer();
		} else { // Start�ł��ĂȂ��C��������Stop���ꂽ�Ȃ�^�C�}��~
			stopTimer();
		}
	}

	// �X�^�[�g
	void start() {
		try {
			// RT�R���|�[�l���g�N��
			robotSync = (RobotSync*) manager->createComponent("RobotSync");
			m_Mode = MODE_START;
			info("Successfully started.");
		} catch (std::exception& ex) {
			error("Start faled.");
			print_exc(ex.what());
			return;
		}
	}

	// �X�g�b�v
	void stop() {
		// RT�R���|�[�l���g��~
		robotSync->exit();
		m_Mode =MODE_STOP;
		info("Successfully stopped.");
	}

	// Connect�{�^���������ꂽ��
	void onConnectButtonClicked() {
		trace("Connect Button Clicked.");
		try {
			if(isStarted() && !isConnected()) { // �X�^�[�g����Connect���ĂȂ��Ȃ�Connect
				connect();
			} else if(isStarted() && isConnected()) { // �X�^�[�g����Connect���Ă���Disconnet
				disconnect();
			}
		} catch (std::exception& ex) {
			error("Connection Failed.");
			print_exc(ex.what());
		}
	}

	// Connect����
	void connect() {
		bool foundFlag = false; 
		ItemList<BodyItem> bodyItems =
		ItemTreeView::mainInstance()->checkedItems<BodyItem>();
		for(size_t i = 0;i < bodyItems.size(); ++i) {
			BodyItemPtr body = bodyItems[i];
			if(body->name() == "robot") {
				m_RobotBodyItem = body;
				foundFlag = true;
				//debug3("ang_min_max", m_RobotBodyItem->body()->numJoints(), robotSync->m_default_mask.size(), robotSync->m_default_angle_radian.size());
			}
		}

		if(!foundFlag) {
			info("RobotSyncPlugin can not find 'robot' model in the work tree. Import robot model and name it as 'robot'");
			return;
		}

		robotSync->m_out.data.length(m_RobotBodyItem->body()->numJoints());

		info("Activating RTC....");
		robotSync->activate(0); // RTC���A�N�e�B�u��
		while(true) {
			// �A�N�e�B�u�ɂȂ�܂ő҂D
			RTC::LifeCycleState state = robotSync->getExecutionContext(0)->get_component_state(robotSync->getObjRef());
			if(state == RTC::ACTIVE_STATE) {
				break;
			} else if(state == RTC::ERROR_STATE) {
				info("Activating RTC failed.");
				return;
			}
		}
		info("RTC is successfully activated.");


		info("RobotSyncPlugin successfully connected to RTC (RobotSync*.rtc)");
		m_Mode = MODE_CONNECTED;
	}

	// Disconnect
	void disconnect() {
		m_Mode = MODE_START;
	}


	// ��Ԋm�F
	bool isStarted() {
		return m_Mode == MODE_START;
	}

	// ��Ԋm�F
	bool isConnected() {
		return m_Mode == MODE_CONNECTED;
	}

	// �^�C�}��~
	void stopTimer(void) {
		m_Timer.stop();
	}

	// �^�C�}�J�n
	void startTimer(void) {
		m_Timer.start(100); // �C���^�[�o����100 (10Hz) �Ƃ���D
	}

	// �^�C�}�ɂ���������
	void onTimer(void) {
		if(isConnected()) {
			// ���f������e�֐߂̊p�x���擾
			for(int i = 0;i < m_RobotBodyItem->body()->numJoints();i++) {
				if(robotSync->m_default_mask[i] == 0) { // �}�X�N���[���Ȃ烂�f���̃f�[�^���o��
					robotSync->m_out.data[i] = m_RobotBodyItem->body()->joint(i)->q();
				} else { // �}�X�N��0����Ȃ��Ȃ�C�f�t�H���g�l���o��
					robotSync->m_out.data[i] = robotSync->m_default_angle_radian[i];
				}
			}
			// �f�[�^�|�[�g�ɏ�������
			robotSync->m_outOut.write();
		} else {
			if(!isStarted()) {
				stopTimer();
			}
		}
	}

private:
	cnoid::Timer m_Timer;

    ROBOT_CONTROLLER_MODE m_Mode;
	cnoid::ToolButton* m_pStartButton;
	cnoid::ToolButton* m_pConnectButton;

	cnoid::BodyItemPtr m_RobotBodyItem;

	boost::thread rtcManagerMainLoopThread;

	RobotSync* robotSync;
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotSyncPlugin)
