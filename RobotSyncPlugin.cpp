/**
 * @author Yuki Suga (SSR) ysuga@ysuga.net
 * 
 * Choreonoid内のモデルのデータをRobotSyncという名前のRTCのDoubleSeq
 * 
 * まず，モデルの名前はrobotでなければならない．
 * これは，cnoidファイルをテキストエディタで直接変更すれば，変更できる．
 *
 * StartするとRobotSync*.rtcというRTコンポーネントを作成
 * Connectすると，RTCを自らアクティブ化し，シミュレータ内のモデルの間接角度を出力する．
 *
 * RTCには，default_maskとdefault_angle_radianというコンフィグレーションがあり，
 * これらはCSV形式の文字列で，onActivatedで数値配列として解釈される．
 * default_mask, default_angle_radianの両方のCSVで渡される要素の数は，ロボットモデルの関節の数と同じでなければならない．
 * たとえば，関節数が3のモデルであれば，maskは1,1,0で，angle_radianは1.0, 1.58, 0.9だったとする．
 * もし，関節数が3でなければ，このRTCはConnectしようとするとエラーとなる．
 * 要素数が合えばActive化される．
 *
 * default_maskは，default_angle_radianで指定されるデフォルト角度値のみ出力するマスク関節か，否かを決めるフラグで，
 * 1を指定された関節は，default_angle_radianの角度のみを出力する．
 * 0を指定されると，choreonoid上のモデルの角度を出力する．
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


// ここからデバッグ関連のマクロの定義
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
// デバッグ関連マクロ定義はここまで


// プラグインの状態
enum ROBOT_CONTROLLER_MODE {
	MODE_START, // RTCが起動中
	MODE_STOP, // RTCが停止中
	MODE_CONNECTED, // RTCとシミュレータ内のモデルが接続されており，周期的に目標角度を吐き出す状態
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
	 * ここで初期化
	 */
	virtual bool initialize()
	{
		// Robotという名前のツールバーを作る．
		ToolBar* bar = new ToolBar("Robot");
		m_pStartButton = bar->addButton("Start/Stop"); // Start/Stopボタンを追加
		m_pStartButton->sigClicked().connect(bind(&RobotSyncPlugin::onStartButtonClicked, this));
		m_pConnectButton = bar->addButton("Connect/Disconnect"); // Connect/Disconnectボタンを追加
		m_pConnectButton->sigClicked().connect(bind(&RobotSyncPlugin::onConnectButtonClicked, this));
		addToolBar(bar);

		// タイマーを作成．周期実行の準備
		m_Timer.sigTimeout().connect(bind(&RobotSyncPlugin::onTimer, this));

		info("RobotSyncPlugin is successfully initialized.");	

		// 停止状態にしておく
		m_Mode = MODE_STOP;

		// RTコンポーネントの起動準備．この時点でネームサービスが起動していないとアウト．
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
		// Managerの起動
        manager = RTC::Manager::init(numArgs, const_cast<char**>(argv));
        manager->activateManager();
            
#ifdef Q_OS_WIN32
        omniORB::setClientCallTimeout(0); // reset the global timeout setting?
#endif
		// ManagerにRTコンポーネントを登録
		RobotSync::registerFactory(manager, "RobotSync");

		// Managerの実行は別スレッドから実行したいのでスレッドを立てる．
		rtcManagerMainLoopThread = boost::thread(rtcManagerMainLoop);


		return true;
	}

	// Startボタンが押された時
	void onStartButtonClicked() {
		trace("Start Button Clicked.");
		if(isStarted()) { // StartしてたらStop
			if(isConnected()) { // もしConnectedならStopできない．
				info("RobotSync Plugin is already connected to robot model. Can not stop.");
			} else {
				stop();
			}
		} else { // StopしてたらStart
			start();
		}
		if(isStarted()) { // Startできてたらタイマ起動
			startTimer();
		} else { // Startできてない，もしくはStopされたならタイマ停止
			stopTimer();
		}
	}

	// スタート
	void start() {
		try {
			// RTコンポーネント起動
			robotSync = (RobotSync*) manager->createComponent("RobotSync");
			m_Mode = MODE_START;
			info("Successfully started.");
		} catch (std::exception& ex) {
			error("Start faled.");
			print_exc(ex.what());
			return;
		}
	}

	// ストップ
	void stop() {
		// RTコンポーネント停止
		robotSync->exit();
		m_Mode =MODE_STOP;
		info("Successfully stopped.");
	}

	// Connectボタンが押された時
	void onConnectButtonClicked() {
		trace("Connect Button Clicked.");
		try {
			if(isStarted() && !isConnected()) { // スタートしてConnectしてないならConnect
				connect();
			} else if(isStarted() && isConnected()) { // スタートしてConnectしてたらDisconnet
				disconnect();
			}
		} catch (std::exception& ex) {
			error("Connection Failed.");
			print_exc(ex.what());
		}
	}

	// Connect処理
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
		robotSync->activate(0); // RTCをアクティブ化
		while(true) {
			// アクティブになるまで待つ．
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


	// 状態確認
	bool isStarted() {
		return m_Mode == MODE_START;
	}

	// 状態確認
	bool isConnected() {
		return m_Mode == MODE_CONNECTED;
	}

	// タイマ停止
	void stopTimer(void) {
		m_Timer.stop();
	}

	// タイマ開始
	void startTimer(void) {
		m_Timer.start(100); // インターバルを100 (10Hz) とする．
	}

	// タイマによる周期処理
	void onTimer(void) {
		if(isConnected()) {
			// モデルから各関節の角度を取得
			for(int i = 0;i < m_RobotBodyItem->body()->numJoints();i++) {
				if(robotSync->m_default_mask[i] == 0) { // マスクがゼロならモデルのデータを出力
					robotSync->m_out.data[i] = m_RobotBodyItem->body()->joint(i)->q();
				} else { // マスクが0じゃないなら，デフォルト値を出力
					robotSync->m_out.data[i] = robotSync->m_default_angle_radian[i];
				}
			}
			// データポートに書き込む
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
