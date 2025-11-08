/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>  // 👈
#include <Encoder.h>
#include "switch.h" // 👈 追加
#include "LED.h"    // 👈 追加
#include "motor.h"         // 👈 追加
#include "VelocityCtrl.h"  // 👈 追加
#include "LineChase.h"     // 👈 追加

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LINE_SENSOR_NUM 16 // センサーの数を16個に定義

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//どのモードで走行するかを後々格納する
volatile uint16_t run_Mode = 0;
//走行中かどうか(trueで走行)
volatile bool running = false;
//システム起動からの経過時間
volatile uint32_t systemTime = 0;
//走行開始からの経過時間
volatile uint32_t runningTime = 0;

volatile bool side_sensor_L = false;          // 左サイドセンサの状態
volatile bool side_sensor_R = false;          // 右サイドセンサの状態
volatile uint8_t goal_logic_state = 0;       // ゴール判定のステートマシン (先輩の 'pattern' 相当)
volatile uint16_t start_goal_line_cnt = 0;    // 右ラインの通過回数
volatile bool goal_judge_flag = false;      // ゴール判定の補助フラグ
volatile bool is_goal = false;                // ゴールが確定したかどうかのフラグ

volatile bool cross_line_ignore_flag = false; // クロスラインを無視する期間か

int SW2_prev = 1;
int SW2_current = 1;
int SW3_prev = 1;
int SW3_current = 1;

volatile uint8_t robot_mode = 0; // ロボットの状態 (0:キャリブレーション待機, 1:走行待機, 2:走行中)
volatile uint8_t sw_state = 0;   // スイッチの状態を保持する変数

int M_R_drive = 0;
int M_L_drive = 0;
int M_L = 0;
int M_R = 0;

int LED = 0;

int WARIKOMI_cnt = 0;

volatile uint16_t analog[16];

int32_t Enc_L = 0;
int32_t Enc_R = 0;
//キャリブレーション用
float max_values[LINE_SENSOR_NUM];
float min_values[LINE_SENSOR_NUM];
float sensor_coefficient[LINE_SENSOR_NUM];
float black_values[LINE_SENSOR_NUM];
// 正規化後のセンサー値 (0〜1000)
volatile int16_t Linesensor[LINE_SENSOR_NUM];

// エンコーダデバッグ用のグローバル変数--------------------------------
int16_t debug_encoder_l = 0;
int16_t debug_encoder_r = 0;
float debug_velocity = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//---------------------------一旦けしていいらしい（Encorder.hと重複するから）
//void updateEncoderCnt(void) {
//	// __HAL_TIM_GET_COUNTER を使ってタイマーの現在値を取得
//	// ※TIM3とTIM4のどちらが左右かは、基板の配線によります
//	Enc_L = __HAL_TIM_GET_COUNTER(&htim3);
//	Enc_R = __HAL_TIM_GET_COUNTER(&htim4);
//
//	// (カウント値をリセットする場合はここでリセットする)
//	// __HAL_TIM_SET_COUNTER(&htim3, 0);
//	// __HAL_TIM_SET_COUNTER(&htim4, 0);
//}
void sensorCalibration() {
	float max_values_buffer[LINE_SENSOR_NUM] = { 0 };
	float min_values_buffer[LINE_SENSOR_NUM] = { 1500 }; // (先輩のコードに合わせる)

	for (uint16_t i = 0; i < LINE_SENSOR_NUM; i++) {
		max_values[i] = 0;
		min_values[i] = 1500;
	}

	// (サイドセンサーの部分は一旦省略)

	while (getSwitchStatus('L') == 1) { // 左スイッチが押されている間
		setLED('R'); // (LED.cの機能)

		for (uint16_t i = 0; i < LINE_SENSOR_NUM; i++) {
			max_values_buffer[i] = analog[i]; // 👈 adc_value を Line_Sensor に変更
			min_values_buffer[i] = analog[i]; // 👈 adc_value を Line_Sensor に変更

			if (max_values_buffer[i] > max_values[i]) {
				max_values[i] = max_values_buffer[i];
			}
			if ((min_values_buffer[i] < min_values[i])) {
				min_values[i] = min_values_buffer[i];
			}
		}
	}

	for (uint16_t i = 0; i < LINE_SENSOR_NUM; i++) {
		sensor_coefficient[i] = max_values[i] - min_values[i];
		black_values[i] = min_values[i];
	}

	// (サイドセンサーの部分は一旦省略)
}

void updateLineSensor(void) {

	// 正規化 (0〜1000)
	Linesensor[0] = ((analog[0] - black_values[0]) / sensor_coefficient[0])
			* 1000;
	Linesensor[1] = ((analog[1] - black_values[1]) / sensor_coefficient[1])
			* 1000;
	Linesensor[2] = ((analog[2] - black_values[2]) / sensor_coefficient[2])
			* 1000;
	Linesensor[3] = ((analog[3] - black_values[3]) / sensor_coefficient[3])
			* 1000;
	Linesensor[4] = ((analog[4] - black_values[4]) / sensor_coefficient[4])
			* 1000;
	Linesensor[5] = ((analog[5] - black_values[5]) / sensor_coefficient[5])
			* 1000;
	Linesensor[6] = ((analog[6] - black_values[6]) / sensor_coefficient[6])
			* 1000;
	Linesensor[7] = ((analog[7] - black_values[7]) / sensor_coefficient[7])
			* 1000;
	Linesensor[8] = ((analog[8] - black_values[8]) / sensor_coefficient[8])
			* 1000;
	Linesensor[9] = ((analog[9] - black_values[9]) / sensor_coefficient[9])
			* 1000;
	Linesensor[10] = ((analog[10] - black_values[10]) / sensor_coefficient[10])
			* 1000;
	Linesensor[11] = ((analog[11] - black_values[11]) / sensor_coefficient[11])
			* 1000;
	Linesensor[12] = ((analog[12] - black_values[12]) / sensor_coefficient[12])
			* 1000;
	Linesensor[13] = ((analog[13] - black_values[13]) / sensor_coefficient[13])
			* 1000;
	Linesensor[14] = ((analog[14] - black_values[14]) / sensor_coefficient[14])
			* 1000;
	Linesensor[15] = ((analog[15] - black_values[15]) / sensor_coefficient[15])
			* 1000;

	// クリップ処理 (0未満や1000より大きい値を丸める)
	for (int j = 0; j < LINE_SENSOR_NUM; j++) {
		if (Linesensor[j] >= 1000)
			Linesensor[j] = 1000;
		if (Linesensor[j] <= 0)
			Linesensor[j] = 0;
	}
}

void updateLinesensorCnt(void) {

}

void updateSideSensors(void) {
	// GPIOピンを読む (あなたのピン定義 に合わせる)
	// (LOWで反応 = 白線 と仮定)
	if (HAL_GPIO_ReadPin(Side_sensor2_GPIO_Port, Side_sensor2_Pin)
			== GPIO_PIN_RESET) {
		side_sensor_L = true; // 左が白線
	} else {
		side_sensor_L = false;
	}

	if (HAL_GPIO_ReadPin(Side_sensor1_GPIO_Port, Side_sensor1_Pin)
			== GPIO_PIN_RESET) {
		side_sensor_R = true; // 右が白線
	} else {
		side_sensor_R = false;
	}

}

bool isCrossLine(void) {
	static uint16_t cnt = 0;
	// センサ 1個目 (Linesensor[0]) と 16個目 (Linesensor[15]) をチェック
	float sensor_edge_val_l = Linesensor[0];
	float sensor_edge_val_r = Linesensor[15];
	static bool flag = false;

	// 正規化後は「白 = 1000」 なので、 200以上を「白線」とみなす
	if (sensor_edge_val_l < 200 && sensor_edge_val_r < 200) {
		cnt++;
	} else {
		cnt = 0;
	}

	if (cnt >= 5) { // 10ms連続で両端が白ならクロスと判定
		flag = true;
	} else {
		flag = false;
	}
	return flag;
}

void checkGoalLogic(void) {

	// --- 1. クロス無視タイマーの管理 (70mmで解除) ---
	if (cross_line_ignore_flag == true && getCrossLineIgnoreDistance() >= 70) {
		cross_line_ignore_flag = false; // 無視フラグを解除
	}

	// --- 2. ゴール判定ステートマシン (最優先で実行) ---
	switch (goal_logic_state) {

	case 0: // 【状態0: スタートライン待ち】
		// (無視期間中 *ではない* 時に、右センサが反応したら)
		if (cross_line_ignore_flag == false && side_sensor_R == true) {
			start_goal_line_cnt = 1;      // 1回目をカウント
			clearGoalJudgeDistance();     // 距離リセット
			goal_logic_state = 5;         // 次の状態へ

			// スタートライン自体も「クロス」として扱い、無視タイマーを開始
			cross_line_ignore_flag = true;
			clearCrossLineIgnoreDistance();
		}
		break;

	case 5: // 【状態5: スタートラインを抜けきるのを待つ】
		if (side_sensor_R == false) { // 右センサが途切れたら
			goal_logic_state = 10;    // 次の状態へ
		}
		break;

	case 10: // 【状態10: 走行中（ゴール待ち）】

		// --- ★ A. 「ゴール判定」 (最優先) ★ ---
		// (クロス無視期間中 *ではない* 時に)
		// (ゴール候補フラグが *立っておらず*)
		// (右センサが反応し) かつ (70mm以上経過していたら)
		if (cross_line_ignore_flag == false && goal_judge_flag == false
				&& side_sensor_R == true && getGoalJudgeDistance() >= 70) {

			goal_judge_flag = true; // ゴール候補
			clearGoalJudgeDistance();
		}
		// (ゴール候補のまま、さらに70mm進んだら ＝ ゴールライン確定)
		else if (goal_judge_flag == true && getGoalJudgeDistance() >= 70) {
			start_goal_line_cnt = 2; // 2回目をカウント
			goal_judge_flag = false;
			clearGoalJudgeDistance();
		}

		// 2回カウントしたらゴール状態へ
		if (start_goal_line_cnt >= 2) {
			goal_logic_state = 20;    // 停止状態へ
			break; // 👈 ゴール確定。以下のクロス判定は実行しない
		}
		// ---

		// --- ★ B. 「(ゴールでなければ) クロス判定」 ★ ---
		// (要望のあった isCrossLine() と、既存の side_sensor_L の両方で判定)
		if ((isCrossLine() == true || side_sensor_L == true)
				&& cross_line_ignore_flag == false) {

			// どちらかが反応し、かつ無視期間中でなければ
			cross_line_ignore_flag = true;    // 👈 無視期間スタート
			clearCrossLineIgnoreDistance(); // 👈 タイマースタート
			goal_judge_flag = false;        // 👈 ゴール判定をリセット
			clearGoalJudgeDistance();     // 👈 ゴール距離タイマーもリセット
		}
		break;
		// ---

	case 20: // 【状態20: ゴール検知】
		is_goal = true; // 👈 ゴールフラグを立てる
		break;
	}
}

/**
 * @brief ゴールフラグを返す
 */
bool getGoalStatus(void) {
	return is_goal;
}

/**
 * @brief ゴールロジックを初期化する (走行開始時に呼ぶ)
 */
void initGoalLogic(void) {
	goal_logic_state = 0;
	start_goal_line_cnt = 0;
	goal_judge_flag = false;
	is_goal = false;
	clearGoalJudgeDistance();

	cross_line_ignore_flag = false;
	clearCrossLineIgnoreDistance();
}

void debugEncoder(void) {
	// エンコーダの値を取得してグローバル変数に格納
	getEncoderCnt(&debug_encoder_l, &debug_encoder_r);

	// 現在の速度も取得（オプション）
	debug_velocity = getCurrentVelocity();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {//----------------------------------------
	/* ==============================================
	 * 1ms周期の「メイン制御」 (現場監督)
	 * ============================================== */
	if (htim->Instance == TIM6) {
		// --- 1. タイマー更新 ---
		systemTime++; // システムタイマーを1ms進める

		// +++ ステップ1動作確認用 +++

		// --- 2. センサー値の更新 (毎回実行) ---

		updateEncoderCnt();   // 👈 これを追加！
		updateLineSensor(); // 👈 これを追加！
		updateSideSensors();

		// デバッグ情報の更新
		debugEncoder();

		// updateIMUValue();     // (ジャイロセンサーがあれば)

		// --- 3. 走行制御 (フラグがtrueの時だけ実行) ---
		if (running == true) {
			// 走行開始からの時間をカウント
			runningTime++;

			// [A] 制御計算
			calculateLineFollowingTermFlip(); // 👈 移植した関数を呼ぶ
			calculateVelocityControlFlip(); // 👈 移植した関数を呼ぶ

			// calculateAngleControlFlip();

			// [B] 実行
			lineTraceFlip(); // 👈 移植した関数を呼ぶ
//			runningFlip();   // 👈 移植した関数を呼ぶ
			motorCtrlFlip(); // 👈 移植した関数を呼ぶ

			// [C] その他
			// 吸引モーター

			// コースアウト検知
			checkCourseOut(); // 👈 存在する関数を呼び出す
			checkGoalLogic();
			if (getCouseOutFlag() == true || getGoalStatus() == true) { // 👈 ★★★ このように変更 ★★★
				running = false; // 👈 runningフラグを倒す
			}

		} else {
			// --- 4. 停止処理 (フラグがfalseの時) ---

			// 走行開始からのタイマーをリセット
			runningTime = 0;

			stopVelocityControl(); //
			stopLineTrace();       //
			motorCtrlFlip();
			// (モーターを確実に停止させる関数をここに呼ぶのが安全)

			HAL_TIM_Base_Stop_IT(&htim6); // 1msタイマー自体を停止これいるか？？？
			// stopMotor(); // (例: motorCtrlFlip(0, 0) をラップした関数)
		}
	}
}

// (init関数などもここに配置できます)
void init(void) {
	// ... 初期化処理 ...

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analog, 16) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	HAL_Delay(50);

//	sensorCalibration();

//	clearspeedcount();
//	setTargetVelocity(-1.5);

//	setrunmode(1);

//	startVelocityControl();
//	startLineTrace(); //

	initMotor(); // 👈 追加

	// タイマー割り込みを開始
//	HAL_TIM_Base_Start_IT(&htim6);

	// ... その他の初期化 ...
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM8_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	init();

	//基板に部品実装当時の同さチェック用コード----------------------------------
//	if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
//		Error_Handler();
//	}
//	if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) {
//		Error_Handler();
//	}
//
//	HAL_GPIO_WritePin(M_L_PH_GPIO_Port, M_L_PH_Pin, SET);
//	HAL_GPIO_WritePin(M_R_PH_GPIO_Port, M_R_PH_Pin, SET);
//
//	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
//	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
//	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
//	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, SET);
//	HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, SET);
	//----------------------------------------------------------------

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// 1. スイッチ状態を読み取る (switch.cの関数)
		// (もし `getSwitchState` が無ければ `sw_state = getSwitchStatus('L');` で代用)
		sw_state = getSwitchStatus('L');

		// --- ゴール/脱線チェック ---
		// 1ms割り込みが 'running=false' にしたら、モードを「走行待ち」に戻す
		if (robot_mode == 2 && running == false) {
			robot_mode = 1; // 走行待ち状態に戻る
		}

		// --- ロボットの状態で処理を分岐 ---
		switch (robot_mode) {

		case 0: // 【状態0: キャリブレーション待ち】
			setLED('W'); // (例: 白LED点灯)

			// (sw_state & 0x01) は「左スイッチが押されている」
			if (sw_state == 1) {
				setLED('R'); // (赤色LED点灯)

				// 2. キャリブレーション実行 (140行目 の関数。スイッチが離されるまで待機する)
				sensorCalibration();

				// 3. キャリブレーション完了 (スイッチが離された)
				robot_mode = 1; // 👈 「走行待ち」状態へ
				setLED('Y'); // (黄色LED点灯)
				HAL_Delay(200);  // スイッチのチャタリング防止
			}
			break;

		case 1: // 【状態1: 走行待ち (キャリブレーション完了 or 走行停止後)】
			setLED('Y'); // (黄色LED点灯)

			// (sw_state & 0x01) は「左スイッチが押されている」
			if (sw_state & 0x01) {

				// --- 走行準備の "init" をここに集約 ---
				setLED('M'); // (マゼンタLED点灯)

				clearspeedcount();       // 加速ランプをリセット
				setTargetVelocity(-0.8); // 走行速度をセット (マイナスで前進)
				setrunmode(1);           // 速度制御モードをセット

				startVelocityControl(); // 速度制御ON
				startLineTrace();       // ライン追従ON

				// (※※※ 将来、ゴール判定ロジックのリセットをここで行う ※※※)
				initGoalLogic();
				// --- 走行開始 ---
				running = true; // 👈 1ms割り込み内の制御を有効化
				HAL_TIM_Base_Start_IT(&htim6); // 👈 1msタイマーをスタート

				robot_mode = 2; // 👈 「走行中」状態へ
				HAL_Delay(200); // スイッチのチャタリング防止
			}
			break;

		case 2: // 【状態2: 走行中】
			// 1ms割り込みが 'running=true' で走行中。
			// 'while(1)' ループは、'running' が 'false' になるのを待つだけ。
			// (LEDは緑のまま)
			break;
		}

		HAL_Delay(10); // while(1)ループを少し遅くする (CPU負荷軽減とチャタリング防止)

		/* (古いデバッグコードは削除) */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 16;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 7;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 8;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 9;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 10;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 11;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 12;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 13;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 14;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 15;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 16;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 83;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1399;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	LED_1_Pin | LED_2_Pin | M_L_PH_Pin | FLED_B_Pin | FLED_G_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_3_Pin | CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, M_R_PH_Pin | LED_4_Pin | LED_5_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(FLED_R_GPIO_Port, FLED_R_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_1_Pin LED_2_Pin M_L_PH_Pin FLED_B_Pin
	 FLED_G_Pin */
	GPIO_InitStruct.Pin = LED_1_Pin | LED_2_Pin | M_L_PH_Pin | FLED_B_Pin
			| FLED_G_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : vbas_Pin */
	GPIO_InitStruct.Pin = vbas_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(vbas_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Side_sensor1_Pin Side_sensor2_Pin */
	GPIO_InitStruct.Pin = Side_sensor1_Pin | Side_sensor2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_3_Pin CS_Pin */
	GPIO_InitStruct.Pin = LED_3_Pin | CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SCK_Pin MISO_Pin MOSI_Pin */
	GPIO_InitStruct.Pin = SCK_Pin | MISO_Pin | MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : M_R_PH_Pin LED_4_Pin LED_5_Pin */
	GPIO_InitStruct.Pin = M_R_PH_Pin | LED_4_Pin | LED_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SW1_Pin */
	GPIO_InitStruct.Pin = SW1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SW2_Pin */
	GPIO_InitStruct.Pin = SW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : FLED_R_Pin */
	GPIO_InitStruct.Pin = FLED_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(FLED_R_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
