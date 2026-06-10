#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

#include <TensorFlowLite.h>

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#define WINDOW_SIZE 8 
#define BUFFER_MASK (WINDOW_SIZE - 1)

float buffer[WINDOW_SIZE];
int head = 0;
float running_sum = 0.0;
float average = 0.0;
float scaled_average = 0.0;

/* From model_development.ipynb
  Scaling Parameters: Mean: 0.0636779359430605, Std: 0.126740584005968
*/

#define training_mean 0.0636779359430605
#define training_std 0.126740584005968
  
constexpr int kTensorArenaSize = 30 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
  
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
  
constexpr int label_count = 2;
const char* labels[label_count] = {"Y", "N"};

extern const unsigned char model_data[];
extern const int model_data_len;

void setup() {
  // Start serial
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Started");

  // Start IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialized IMU!");
    while (1);
  }

  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<2> micro_op_resolver;  // NOLINT
  micro_op_resolver.AddFullyConnected(); // Dense Layer
  micro_op_resolver.AddLogistic(); // Sigmoid is considered a logistic function

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  interpreter->AllocateTensors();

  // Get model input tensor
  TfLiteTensor* model_input = interpreter->input(0);

  /* From model_validation.ipynb: 
    Input shape expected by TFLite: [1 1]
    Input dtype expected by TFLite: <class 'numpy.float32'>
    Output shape: [1 1]
  */
  if ((model_input->dims->size != 2) ||
      (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != 1) ||
      (model_input->type != kTfLiteFloat32)) {
    TF_LITE_REPORT_ERROR(error_reporter,"Bad input tensor parameters in model");
    return;
  }
  TfLiteTensor* model_output = interpreter->output(0);
  if ((model_output->dims->size != 2) ||
      (model_output->dims->data[0] != 1) ||
      (model_output->dims->data[1] != 1) ||
      (model_output->type != kTfLiteFloat32)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Bad output tensor parameters in model");
    return;
  }

}

void loop() {
  
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
 
  running_sum -= buffer[head];
  buffer[head] = ax * ax;
  running_sum += buffer[head];
  head = (head + 1) & BUFFER_MASK;
  average = running_sum / WINDOW_SIZE;
  scaled_average = (average - training_mean) / training_std;



  // Pass to the model and run the interpreter
  TfLiteTensor* model_input = interpreter->input(0);
  model_input->data.f[0] = scaled_average;  
  
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
    return;
  }
  TfLiteTensor* output = interpreter->output(0);

  // Parse and interpret the model output

  float probability = output->data.f[0];
  const int predicted_class = (probability >= 0.5f) ? 1 : 0;

  char line[30];
  snprintf(line,sizeof(line),"%.3f|%.3f|%s", ax, average, predicted_class == 1 ? "Y" : "N");
  Serial.println(line);
  delay(10);
}