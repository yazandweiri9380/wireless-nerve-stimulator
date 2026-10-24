#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "BurstMode.h"

// ============================================================
// Apollo3-friendly compact fully-connected ANN latency benchmark
// ------------------------------------------------------------
// Features:
// - 3 input neurons
// - 16 output neurons
// - outputs quantized to integers in [-127, 128]
// - configurable number of hidden layers
// - configurable neurons per hidden layer
// - fully connected network
// - compact parameter storage: only stores actual weights needed
// - latency benchmark over random inputs
//
// Hidden layers use: Dense + ReLU
// Output layer uses: Dense (linear)
//
// IMPORTANT:
// Edit only the CONFIGURATION section to change architecture.
// ============================================================

// ============================================================
// CONFIGURATION
// ============================================================

const int N_INPUTS = 500;   // reduce if you need less RAM

const int ANN_IN  = 3;
const int ANN_OUT = 16;

// Number of hidden layers actually used
const int NUM_HIDDEN_LAYERS = 8;

// Sizes of hidden layers used, first NUM_HIDDEN_LAYERS entries matter
int HIDDEN_SIZES[] = {128, 128, 64, 33, 32, 16 , 16 , 16};

// Maximum supported hidden layers in this sketch
const int MAX_HIDDEN_LAYERS = 8;

// Repeat inference to improve micros() timing resolution
const int REPEATS = 2;

// ------------------------------------------------------------
// Maximum parameter storage capacity
// ------------------------------------------------------------
// This is the total number of float parameters (weights + biases)
// that can be stored. Adjust upward if you want larger networks,
// but keep in mind RAM usage.
//
// Example:
// 3->64:   64*3 + 64 = 256
// 64->32:  32*64 + 32 = 2080
// 32->16:  16*32 + 16 = 528
// 16->16:  16*16 + 16 = 272
// total = 3136 params
//
// 12000 params = 48 KB for weights/biases
// ============================================================

const int MAX_PARAMS = 46000;

// ============================================================
// Storage
// ============================================================

// Inputs: 3 signed bytes each, range [-100, 100]
int8_t INPUTS[N_INPUTS][ANN_IN];

// Timing arrays
uint32_t ANN_timing_array[N_INPUTS];
uint32_t ANN_sorted[N_INPUTS];

// Parameter buffers
float PARAMS[MAX_PARAMS];

// Layer metadata
// For layer l:
// - weights start at weightOffset[l], size = outSize[l] * inSize[l]
// - biases  start at biasOffset[l],   size = outSize[l]
int inSizeArr [MAX_HIDDEN_LAYERS + 1];
int outSizeArr[MAX_HIDDEN_LAYERS + 1];
int weightOffset[MAX_HIDDEN_LAYERS + 1];
int biasOffset  [MAX_HIDDEN_LAYERS + 1];
int totalLayers = 0;
int totalParams = 0;

// Activation buffers
// Only need size = largest layer width among hidden/output layers
const int MAX_ACTIVATION_SIZE = 256;
float actA[MAX_ACTIVATION_SIZE];
float actB[MAX_ACTIVATION_SIZE];

// Output buffers
float ann_out[ANN_OUT];
int16_t ann_out_int[ANN_OUT];

// Sinks to prevent optimization-out
volatile float ann_sink = 0.0f;
volatile int32_t out_sink = 0;

// ============================================================
// Utility
// ============================================================

int8_t randSignedByte100() {
  long r = random(-100, 101);
  return (int8_t)r;
}

float randFloat(float a, float b) {
  long r = random(0, 10000);
  return a + (b - a) * ((float)r / 9999.0f);
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ============================================================
// Input generation
// ============================================================

void generateInputs() {
  for (int i = 0; i < N_INPUTS; i++) {
    for (int j = 0; j < ANN_IN; j++) {
      INPUTS[i][j] = randSignedByte100();
    }
  }
}

// ============================================================
// Architecture validation and layout
// ============================================================

bool validateArchitecture() {
  if (NUM_HIDDEN_LAYERS < 0 || NUM_HIDDEN_LAYERS > MAX_HIDDEN_LAYERS) {
    Serial.println("ERROR: NUM_HIDDEN_LAYERS out of range.");
    return false;
  }

  for (int i = 0; i < NUM_HIDDEN_LAYERS; i++) {
    if (HIDDEN_SIZES[i] <= 0) {
      Serial.print("ERROR: HIDDEN_SIZES[");
      Serial.print(i);
      Serial.println("] must be > 0.");
      return false;
    }
    if (HIDDEN_SIZES[i] > MAX_ACTIVATION_SIZE) {
      Serial.print("ERROR: HIDDEN_SIZES[");
      Serial.print(i);
      Serial.println("] exceeds MAX_ACTIVATION_SIZE.");
      return false;
    }
  }

  if (ANN_OUT > MAX_ACTIVATION_SIZE) {
    Serial.println("ERROR: ANN_OUT exceeds MAX_ACTIVATION_SIZE.");
    return false;
  }

  return true;
}

int computeRequiredParams() {
  int params = 0;
  int prev = ANN_IN;

  for (int i = 0; i < NUM_HIDDEN_LAYERS; i++) {
    int cur = HIDDEN_SIZES[i];
    params += cur * prev; // weights
    params += cur;        // biases
    prev = cur;
  }

  // Output layer
  params += ANN_OUT * prev;
  params += ANN_OUT;

  return params;
}

bool buildLayerLayout() {
  totalLayers = NUM_HIDDEN_LAYERS + 1;
  int cursor = 0;
  int prev = ANN_IN;

  for (int l = 0; l < NUM_HIDDEN_LAYERS; l++) {
    int cur = HIDDEN_SIZES[l];

    inSizeArr[l] = prev;
    outSizeArr[l] = cur;

    weightOffset[l] = cursor;
    cursor += cur * prev;

    biasOffset[l] = cursor;
    cursor += cur;

    prev = cur;
  }

  // Output layer
  int l = NUM_HIDDEN_LAYERS;
  inSizeArr[l] = prev;
  outSizeArr[l] = ANN_OUT;

  weightOffset[l] = cursor;
  cursor += ANN_OUT * prev;

  biasOffset[l] = cursor;
  cursor += ANN_OUT;

  totalParams = cursor;

  if (totalParams > MAX_PARAMS) {
    Serial.println("ERROR: totalParams exceeds MAX_PARAMS.");
    return false;
  }

  return true;
}

// ============================================================
// Parameter access helpers
// ============================================================

inline float getWeight(int layer, int outNeuron, int inNeuron) {
  int idx = weightOffset[layer] + outNeuron * inSizeArr[layer] + inNeuron;
  return PARAMS[idx];
}

inline void setWeight(int layer, int outNeuron, int inNeuron, float v) {
  int idx = weightOffset[layer] + outNeuron * inSizeArr[layer] + inNeuron;
  PARAMS[idx] = v;
}

inline float getBias(int layer, int outNeuron) {
  int idx = biasOffset[layer] + outNeuron;
  return PARAMS[idx];
}

inline void setBias(int layer, int outNeuron, float v) {
  int idx = biasOffset[layer] + outNeuron;
  PARAMS[idx] = v;
}

// ============================================================
// Initialize parameters
// ============================================================

void initANNParams() {
  for (int l = 0; l < totalLayers; l++) {
    int inSize  = inSizeArr[l];
    int outSize = outSizeArr[l];

    for (int o = 0; o < outSize; o++) {
      for (int i = 0; i < inSize; i++) {
        setWeight(l, o, i, randFloat(-0.5f, 0.5f));
      }
      setBias(l, o, randFloat(-0.1f, 0.1f));
    }
  }
}

// ============================================================
// ANN inference
// Hidden layers: Dense + ReLU
// Output layer: Dense (linear)
// Input normalization: [-100,100] -> [-1.0,1.0]
// ============================================================

void ann_infer_flexible(const int8_t inputRaw[ANN_IN], float out[ANN_OUT]) {
  float inputNorm[ANN_IN];
  for (int i = 0; i < ANN_IN; i++) {
    inputNorm[i] = ((float)inputRaw[i]) / 100.0f;
  }

  // If there are no hidden layers, do direct input->output
  if (NUM_HIDDEN_LAYERS == 0) {
    int l = 0;
    for (int o = 0; o < ANN_OUT; o++) {
      float sum = getBias(l, o);
      for (int i = 0; i < ANN_IN; i++) {
        sum += getWeight(l, o, i) * inputNorm[i];
      }
      out[o] = sum;
    }
    return;
  }

  // First hidden layer: inputNorm -> actA
  {
    int l = 0;
    int inSize = inSizeArr[l];
    int outSize = outSizeArr[l];

    for (int o = 0; o < outSize; o++) {
      float sum = getBias(l, o);
      for (int i = 0; i < inSize; i++) {
        sum += getWeight(l, o, i) * inputNorm[i];
      }
      actA[o] = (sum > 0.0f) ? sum : 0.0f;
    }
  }

  // Remaining hidden layers
  for (int l = 1; l < NUM_HIDDEN_LAYERS; l++) {
    int inSize = inSizeArr[l];
    int outSize = outSizeArr[l];

    for (int o = 0; o < outSize; o++) {
      float sum = getBias(l, o);
      for (int i = 0; i < inSize; i++) {
        sum += getWeight(l, o, i) * actA[i];
      }
      actB[o] = (sum > 0.0f) ? sum : 0.0f;
    }

    for (int i = 0; i < outSize; i++) {
      actA[i] = actB[i];
    }
  }

  // Output layer
  {
    int l = NUM_HIDDEN_LAYERS;
    int inSize = inSizeArr[l];

    for (int o = 0; o < ANN_OUT; o++) {
      float sum = getBias(l, o);
      for (int i = 0; i < inSize; i++) {
        sum += getWeight(l, o, i) * actA[i];
      }
      out[o] = sum;
    }
  }
}

// ============================================================
// Quantize outputs to integers in [-127, 128]
// ============================================================

void quantizeOutputs16(const float in[ANN_OUT], int16_t outInt[ANN_OUT]) {
  for (int i = 0; i < ANN_OUT; i++) {
    int q = (int)lroundf(in[i]);
    q = clampInt(q, -127, 128);
    outInt[i] = (int16_t)q;
  }
}

// ============================================================
// Statistics helpers
// ============================================================

void copyArrayU32(const uint32_t *src, uint32_t *dst, int n) {
  for (int i = 0; i < n; i++) dst[i] = src[i];
}

void sortU32(uint32_t *arr, int n) {
  for (int i = 1; i < n; i++) {
    uint32_t key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

float computeMeanU32(const uint32_t *arr, int n) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) sum += (double)arr[i];
  return (float)(sum / (double)n);
}

float computeStdU32(const uint32_t *arr, int n, float meanVal) {
  if (n < 2) return 0.0f;

  double sumsq = 0.0;
  for (int i = 0; i < n; i++) {
    double d = (double)arr[i] - (double)meanVal;
    sumsq += d * d;
  }
  return (float)sqrt(sumsq / (double)(n - 1));
}

uint32_t percentileU32(const uint32_t *sortedArr, int n, float p) {
  if (p <= 0.0f) return sortedArr[0];
  if (p >= 100.0f) return sortedArr[n - 1];

  float rank = (p / 100.0f) * (float)(n - 1);
  int idx = (int)(rank + 0.5f);
  if (idx < 0) idx = 0;
  if (idx >= n) idx = n - 1;
  return sortedArr[idx];
}

void reportStats(const char *label, const uint32_t *arr, uint32_t *scratch, int n) {
  copyArrayU32(arr, scratch, n);
  sortU32(scratch, n);

  float meanVal = computeMeanU32(arr, n);
  float sdVal   = computeStdU32(arr, n, meanVal);

  uint32_t minVal = scratch[0];
  uint32_t medVal = percentileU32(scratch, n, 50.0f);
  uint32_t p95Val = percentileU32(scratch, n, 95.0f);
  uint32_t p99Val = percentileU32(scratch, n, 99.0f);
  uint32_t maxVal = scratch[n - 1];

  Serial.println();
  Serial.print(label); Serial.println(" latency statistics (microseconds):");
  Serial.print("  n = "); Serial.println(n);
  Serial.print("  mean = "); Serial.println(meanVal, 4);
  Serial.print("  SD = "); Serial.println(sdVal, 4);
  Serial.print("  median = "); Serial.println(medVal);
  Serial.print("  p95 = "); Serial.println(p95Val);
  Serial.print("  p99 = "); Serial.println(p99Val);
  Serial.print("  min = "); Serial.println(minVal);
  Serial.print("  max = "); Serial.println(maxVal);
}

// ============================================================
// Reporting
// ============================================================

void printArchitecture() {
  Serial.println("ANN architecture:");
  Serial.print("  Input neurons: ");
  Serial.println(ANN_IN);

  Serial.print("  Hidden layers: ");
  Serial.println(NUM_HIDDEN_LAYERS);

  for (int i = 0; i < NUM_HIDDEN_LAYERS; i++) {
    Serial.print("    H");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(HIDDEN_SIZES[i]);
  }

  Serial.print("  Output neurons: ");
  Serial.println(ANN_OUT);

  Serial.print("  Total layers (including output): ");
  Serial.println(totalLayers);

  Serial.print("  Total parameters: ");
  Serial.println(totalParams);

  Serial.print("  Parameter bytes (float32): ");
  Serial.println((unsigned long)totalParams * 4UL);

  Serial.print("  REPEATS per timing sample: ");
  Serial.println(REPEATS);

  Serial.println("  Layer details:");
  for (int l = 0; l < totalLayers; l++) {
    Serial.print("    Layer ");
    Serial.print(l);
    Serial.print(": ");
    Serial.print(inSizeArr[l]);
    Serial.print(" -> ");
    Serial.print(outSizeArr[l]);
    Serial.print(" | weights @ ");
    Serial.print(weightOffset[l]);
    Serial.print(" | biases @ ");
    Serial.println(biasOffset[l]);
    enableBurstMode();
  }
}

// ============================================================
// Benchmark
// Measures: inference + quantization
// ============================================================

void benchmarkANN() {
  // Warm-up
  for (int i = 0; i < 100; i++) {
    ann_infer_flexible(INPUTS[i], ann_out);
    quantizeOutputs16(ann_out, ann_out_int);

    for (int o = 0; o < ANN_OUT; o++) {
      ann_sink += ann_out[o];
      out_sink += ann_out_int[o];
    }
  }

  // Timed run
  for (int i = 0; i < N_INPUTS; i++) {
    uint32_t t0 = micros();

    for (int r = 0; r < REPEATS; r++) {
      ann_infer_flexible(INPUTS[i], ann_out);
      quantizeOutputs16(ann_out, ann_out_int);
    }

    uint32_t t1 = micros();

    ANN_timing_array[i] = (t1 - t0) / REPEATS;

    for (int o = 0; o < ANN_OUT; o++) {
      ann_sink += ann_out[o];
      out_sink += ann_out_int[o];
    }
  }
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("Apollo3 compact ANN inference latency benchmark starting...");

  if (!validateArchitecture()) {
    Serial.println("Architecture validation failed. Halting.");
    while (1) { delay(1000); }
  }

  int requiredParams = computeRequiredParams();
  if (requiredParams > MAX_PARAMS) {
    Serial.print("ERROR: requiredParams = ");
    Serial.print(requiredParams);
    Serial.print(" exceeds MAX_PARAMS = ");
    Serial.println(MAX_PARAMS);
    Serial.println("Increase MAX_PARAMS or reduce network size.");
    while (1) { delay(1000); }
  }

  if (!buildLayerLayout()) {
    Serial.println("Layer layout build failed. Halting.");
    while (1) { delay(1000); }
  }

  randomSeed(micros());

  generateInputs();
  initANNParams();

  Serial.println("Inputs and ANN parameters initialized.");
  printArchitecture();

  benchmarkANN();
  reportStats("ANN", ANN_timing_array, ANN_sorted, N_INPUTS);

  Serial.println();
  Serial.print("ANN float sink = ");
  Serial.println(ann_sink, 6);

  Serial.print("ANN int sink = ");
  Serial.println(out_sink);

  ann_infer_flexible(INPUTS[0], ann_out);
  quantizeOutputs16(ann_out, ann_out_int);

  Serial.println();
  Serial.println("Example quantized output[16] in [-127, 128]:");
  for (int i = 0; i < ANN_OUT; i++) {
    Serial.print(ann_out_int[i]);
    if (i < ANN_OUT - 1) Serial.print(", ");
  }
  Serial.println();

  Serial.println("Benchmark complete.");
}

void loop() {
  // no repeated execution
}