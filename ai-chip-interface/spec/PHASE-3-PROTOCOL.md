# WIA-AI-011 Phase 3: Communication Protocols

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 3 establishes protocols for multi-device coordination, inter-chip communication, and distributed execution.

## Topology Discovery

```c
typedef struct {
    int num_devices;
    float bandwidth_gbps[MAX_DEVICES][MAX_DEVICES];
    float latency_us[MAX_DEVICES][MAX_DEVICES];
    wia_interconnect_type_t interconnect[MAX_DEVICES][MAX_DEVICES];
} wia_topology_t;

wia_status_t wia_discover_topology(wia_topology_t* topology);
wia_status_t wia_can_access_peer(wia_device_t dev0, wia_device_t dev1, bool* can_access);
```

## Peer-to-Peer Communication

```c
// Enable P2P access
wia_status_t wia_enable_peer_access(wia_device_t src, wia_device_t dst);
wia_status_t wia_disable_peer_access(wia_device_t src, wia_device_t dst);

// P2P transfers
wia_status_t wia_memcpy_peer(wia_buffer_t dst, wia_buffer_t src, size_t size, wia_stream_t stream);
```

## Collective Operations

```c
// Communicator
typedef struct {
    wia_device_t* devices;
    int num_devices;
    int rank;
} wia_communicator_t;

wia_status_t wia_create_communicator(wia_device_t* devices, int count, wia_communicator_t* comm);

// All-Reduce
typedef struct {
    wia_reduce_op_t op; // SUM, PROD, MIN, MAX, AVG
    wia_datatype_t datatype;
    wia_allreduce_algorithm_t algorithm; // RING, TREE, RABENSEIFNER
} wia_collective_config_t;

wia_status_t wia_all_reduce(wia_communicator_t comm, wia_buffer_t send_buf,
                            wia_buffer_t recv_buf, size_t count,
                            wia_collective_config_t* config, wia_stream_t stream);

// Broadcast
wia_status_t wia_broadcast(wia_communicator_t comm, wia_buffer_t buffer,
                           size_t count, int root, wia_datatype_t dtype, wia_stream_t stream);

// All-Gather
wia_status_t wia_all_gather(wia_communicator_t comm, wia_buffer_t send_buf,
                            wia_buffer_t recv_buf, size_t count,
                            wia_datatype_t dtype, wia_stream_t stream);

// Reduce-Scatter
wia_status_t wia_reduce_scatter(wia_communicator_t comm, wia_buffer_t send_buf,
                                wia_buffer_t recv_buf, size_t count,
                                wia_reduce_op_t op, wia_datatype_t dtype, wia_stream_t stream);
```

## Compression

```c
typedef struct {
    wia_compression_method_t method; // TOPK, THRESHOLD, QUANTIZE
    union {
        struct { float k_ratio; } topk;
        struct { float threshold; } threshold;
        struct { int bits; } quantize;
    } parameters;
    bool error_feedback;
} wia_compression_config_t;

wia_status_t wia_all_reduce_compressed(wia_communicator_t comm, wia_buffer_t send,
                                       wia_buffer_t recv, size_t count,
                                       wia_compression_config_t* compression,
                                       wia_stream_t stream);
```

## RDMA Support

```c
typedef struct {
    int num_nodes;
    int devices_per_node;
    wia_network_backend_t backend; // RDMA, TCP, CUSTOM
    const char* rdma_device;
} wia_distributed_config_t;

wia_status_t wia_create_distributed_communicator(wia_distributed_config_t* config,
                                                 wia_communicator_t* comm);
```

## Synchronization

```c
// Barrier across all devices in communicator
wia_status_t wia_barrier(wia_communicator_t comm, wia_stream_t stream);

// Wait for all operations on all devices
wia_status_t wia_communicator_synchronize(wia_communicator_t comm);
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
