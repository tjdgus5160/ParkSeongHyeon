#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <NIDAQmx.h>
#include <ansi_c.h>
#include <signal.h>

// PID 제어기 구조체 정의
typedef struct {
    double Kp;              // 비례 게인
    double Ki;              // 적분 게인
    double Kd;              // 미분 게인
    double setpoint;        // 목표값
    double integral;        // 적분값 누적
    double prev_error;      // 이전 오차
    double output_min;      // 출력 최소값
    double output_max;      // 출력 최대값
    double sample_time;     // 샘플링 시간(초)
} PIDController;

// 전역 변수
TaskHandle aiTask = 0;
TaskHandle aoTask = 0;
int keepRunning = 1;

// 함수 선언
void PID_Init(PIDController *pid, double Kp, double Ki, double Kd, 
              double setpoint, double output_min, double output_max, 
              double sample_time);
double PID_Compute(PIDController *pid, double input);
double LowPassFilter(double input, double *prev_output, double alpha);
double PID_Compute_Advanced(PIDController *pid, double input);
int DAQmx_Init(TaskHandle *aiTask, TaskHandle *aoTask, const char *deviceName);
void intHandler(int dummy);

// 메인 함수
int main(void) {
    int32 error = 0;
    char errBuff[2048] = {'\0'};
    
    // Ctrl+C 핸들러 설정
    signal(SIGINT, intHandler);
    
    // PID 제어기 초기화
    PIDController pid;
    PID_Init(&pid, 0.5, 0.2, 0.1, 5.0, -10.0, 10.0, 0.001); // 1ms 샘플링 시간
    
    // DAQmx 초기화
    error = DAQmx_Init(&aiTask, &aoTask, "PXIe-6363/ai0");
    if (error < 0) {
        return -1;
    }
    
    // 타이밍 설정 (1kHz 샘플링 레이트)
    DAQmxErrChk(DAQmxCfgSampClkTiming(aiTask, "", 1000.0, DAQmx_Val_Rising, 
                                     DAQmx_Val_ContSamps, 1000));
    
    // 태스크 시작
    DAQmxErrChk(DAQmxStartTask(aiTask));
    DAQmxErrChk(DAQmxStartTask(aoTask));
    
    printf("고속 PID 제어 시작...\n");
    printf("종료하려면 Ctrl+C를 누르세요.\n\n");
    
    // 제어 루프
    while (keepRunning) {
        float64 inputData = 0;
        float64 outputData = 0;
        
        // 아날로그 입력 읽기
        DAQmxErrChk(DAQmxReadAnalogF64(aiTask, 1, 0.01, DAQmx_Val_GroupByChannel, 
                                      &inputData, 1, NULL, NULL));
        
        // PID 제어 계산 (고급 버전 사용)
        outputData = PID_Compute_Advanced(&pid, inputData);
        
        // 아날로그 출력 쓰기
        DAQmxErrChk(DAQmxWriteAnalogF64(aoTask, 1, 1, 0.01, DAQmx_Val_GroupByChannel, 
                                       &outputData, NULL, NULL));
        
        printf("입력: %.3f V, 출력: %.3f V\r", inputData, outputData);
        fflush(stdout);
    }
    
    printf("\n프로그램 종료\n");
    
Error:
    if (DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("DAQmx Error: %s\n", errBuff);
    }
    
    // 태스크 정리
    if (aiTask != 0) {
        DAQmxStopTask(aiTask);
        DAQmxClearTask(aiTask);
    }
    
    if (aoTask != 0) {
        DAQmxStopTask(aoTask);
        DAQmxClearTask(aoTask);
    }
    
    return 0;
}

// PID 제어기 초기화 함수
void PID_Init(PIDController *pid, double Kp, double Ki, double Kd, 
              double setpoint, double output_min, double output_max, 
              double sample_time) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->sample_time = sample_time;
}

// 기본 PID 계산 함수
double PID_Compute(PIDController *pid, double input) {
    // 오차 계산
    double error = pid->setpoint - input;
    
    // 비례항 계산
    double p_term = pid->Kp * error;
    
    // 적분항 계산
    pid->integral += error * pid->sample_time;
    
    // 안티와인드업 - 적분항 제한
    if (pid->integral > pid->output_max / pid->Ki) {
        pid->integral = pid->output_max / pid->Ki;
    } else if (pid->integral < pid->output_min / pid->Ki) {
        pid->integral = pid->output_min / pid->Ki;
    }
    
    double i_term = pid->Ki * pid->integral;
    
    // 미분항 계산
    double derivative = (error - pid->prev_error) / pid->sample_time;
    double d_term = pid->Kd * derivative;
    
    // 이전 오차 업데이트
    pid->prev_error = error;
    
    // PID 출력 계산
    double output = p_term + i_term + d_term;
    
    // 출력 제한
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    return output;
}

// 저역통과 필터 함수
double LowPassFilter(double input, double *prev_output, double alpha) {
    double output = alpha * input + (1.0 - alpha) * (*prev_output);
    *prev_output = output;
    return output;
}

// 고급 PID 계산 함수 (미분항 필터링 포함)
double PID_Compute_Advanced(PIDController *pid, double input) {
    static double prev_derivative = 0.0;
    double filter_alpha = 0.1; // 필터 계수 (0.0 ~ 1.0)
    
    // 오차 계산
    double error = pid->setpoint - input;
    
    // 비례항 계산
    double p_term = pid->Kp * error;
    
    // 적분항 계산
    pid->integral += error * pid->sample_time;
    
    // 안티와인드업 - 적분항 제한
    if (pid->integral > pid->output_max / pid->Ki) {
        pid->integral = pid->output_max / pid->Ki;
    } else if (pid->integral < pid->output_min / pid->Ki) {
        pid->integral = pid->output_min / pid->Ki;
    }
    
    double i_term = pid->Ki * pid->integral;
    
    // 미분항 계산 (저역통과 필터 적용)
    double derivative = (error - pid->prev_error) / pid->sample_time;
    derivative = LowPassFilter(derivative, &prev_derivative, filter_alpha);
    double d_term = pid->Kd * derivative;
    
    // 이전 오차 업데이트
    pid->prev_error = error;
    
    // PID 출력 계산
    double output = p_term + i_term + d_term;
    
    // 출력 제한
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    return output;
}

// DAQmx 초기화 함수
int DAQmx_Init(TaskHandle *aiTask, TaskHandle *aoTask, const char *deviceName) {
    int32 error = 0;
    char errBuff[2048] = {'\0'};
    
    // 아날로그 입력 태스크 생성
    DAQmxErrChk(DAQmxCreateTask("", aiTask));
    
    // 아날로그 입력 채널 설정 (AI0)
    DAQmxErrChk(DAQmxCreateAIVoltageChan(*aiTask, 
                                        deviceName, 
                                        "AI_Channel", 
                                        DAQmx_Val_Cfg_Default, 
                                        -10.0, 10.0, 
                                        DAQmx_Val_Volts, 
                                        NULL));
    
    // 아날로그 출력 태스크 생성
    DAQmxErrChk(DAQmxCreateTask("", aoTask));
    
    // 아날로그 출력 채널 설정 (AO0)
    DAQmxErrChk(DAQmxCreateAOVoltageChan(*aoTask, 
                                        deviceName, 
                                        "AO_Channel", 
                                        -10.0, 10.0, 
                                        DAQmx_Val_Volts, 
                                        NULL));
    
    return 0;
    
Error:
    if (DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("DAQmx Error: %s\n", errBuff);
        
        if (*aiTask != 0) {
            DAQmxStopTask(*aiTask);
            DAQmxClearTask(*aiTask);
        }
        
        if (*aoTask != 0) {
            DAQmxStopTask(*aoTask);
            DAQmxClearTask(*aoTask);
        }
    }
    
    return error;
}

// Ctrl+C 핸들러
void intHandler(int dummy) {
    keepRunning = 0;
}
