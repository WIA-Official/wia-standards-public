# WIA-SEMI-004 API Reference

## Core APIs

### Device Management
```cpp
Status initialize()
Status shutdown()
Status getDeviceInfo(DeviceInfo* info)
```

### Model Operations
```cpp
Status loadModel(const char* path, ModelHandle* handle)
Status inference(ModelHandle handle, Tensor* inputs, Tensor* outputs)
Status unloadModel(ModelHandle handle)
```

### Performance Monitoring
```cpp
Status getMetrics(PerformanceMetrics* metrics)
```

---
*© 2025 WIA*
