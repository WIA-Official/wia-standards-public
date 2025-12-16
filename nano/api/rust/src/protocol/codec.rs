//! Message codec for serialization/deserialization

use crate::error::{NanoError, NanoResult};
use super::ProtocolMessage;

/// Codec for message serialization
pub trait MessageCodec: Send + Sync {
    /// Encode a message to bytes
    fn encode(&self, message: &ProtocolMessage) -> NanoResult<Vec<u8>>;

    /// Decode bytes to a message
    fn decode(&self, data: &[u8]) -> NanoResult<ProtocolMessage>;

    /// Get codec name
    fn name(&self) -> &'static str;
}

/// JSON codec (default)
pub struct JsonCodec {
    pretty: bool,
}

impl JsonCodec {
    pub fn new() -> Self {
        Self { pretty: false }
    }

    pub fn pretty() -> Self {
        Self { pretty: true }
    }
}

impl Default for JsonCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl MessageCodec for JsonCodec {
    fn encode(&self, message: &ProtocolMessage) -> NanoResult<Vec<u8>> {
        let json = if self.pretty {
            serde_json::to_vec_pretty(message)
        } else {
            serde_json::to_vec(message)
        };

        json.map_err(|e| NanoError::Serialization(e))
    }

    fn decode(&self, data: &[u8]) -> NanoResult<ProtocolMessage> {
        serde_json::from_slice(data).map_err(|e| NanoError::Serialization(e))
    }

    fn name(&self) -> &'static str {
        "json"
    }
}

/// Compact binary codec using MessagePack
pub struct BinaryCodec;

impl BinaryCodec {
    pub fn new() -> Self {
        Self
    }
}

impl Default for BinaryCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl MessageCodec for BinaryCodec {
    fn encode(&self, message: &ProtocolMessage) -> NanoResult<Vec<u8>> {
        // Use JSON as fallback (in real impl, would use msgpack or similar)
        serde_json::to_vec(message).map_err(|e| NanoError::Serialization(e))
    }

    fn decode(&self, data: &[u8]) -> NanoResult<ProtocolMessage> {
        serde_json::from_slice(data).map_err(|e| NanoError::Serialization(e))
    }

    fn name(&self) -> &'static str {
        "binary"
    }
}

/// Codec with optional compression
pub struct CompressedCodec<C: MessageCodec> {
    inner: C,
    compression_threshold: usize,
}

impl<C: MessageCodec> CompressedCodec<C> {
    pub fn new(inner: C) -> Self {
        Self {
            inner,
            compression_threshold: 256, // Compress if > 256 bytes
        }
    }

    pub fn with_threshold(mut self, threshold: usize) -> Self {
        self.compression_threshold = threshold;
        self
    }

    fn compress(data: &[u8]) -> Vec<u8> {
        // Simple RLE-like compression (in real impl, would use zlib/lz4)
        let mut compressed = Vec::new();
        compressed.push(1); // Compression flag

        let mut i = 0;
        while i < data.len() {
            let byte = data[i];
            let mut count = 1;

            while i + count < data.len() && data[i + count] == byte && count < 255 {
                count += 1;
            }

            if count >= 4 {
                compressed.push(0xFF); // Escape
                compressed.push(count as u8);
                compressed.push(byte);
                i += count;
            } else {
                if byte == 0xFF {
                    compressed.push(0xFF);
                    compressed.push(1);
                    compressed.push(byte);
                } else {
                    compressed.push(byte);
                }
                i += 1;
            }
        }

        compressed
    }

    fn decompress(data: &[u8]) -> NanoResult<Vec<u8>> {
        if data.is_empty() {
            return Ok(Vec::new());
        }

        if data[0] != 1 {
            // Not compressed
            return Ok(data.to_vec());
        }

        let mut decompressed = Vec::new();
        let mut i = 1;

        while i < data.len() {
            if data[i] == 0xFF && i + 2 < data.len() {
                let count = data[i + 1] as usize;
                let byte = data[i + 2];
                for _ in 0..count {
                    decompressed.push(byte);
                }
                i += 3;
            } else {
                decompressed.push(data[i]);
                i += 1;
            }
        }

        Ok(decompressed)
    }
}

impl<C: MessageCodec> MessageCodec for CompressedCodec<C> {
    fn encode(&self, message: &ProtocolMessage) -> NanoResult<Vec<u8>> {
        let data = self.inner.encode(message)?;

        if data.len() > self.compression_threshold {
            Ok(Self::compress(&data))
        } else {
            let mut result = vec![0]; // No compression flag
            result.extend(data);
            Ok(result)
        }
    }

    fn decode(&self, data: &[u8]) -> NanoResult<ProtocolMessage> {
        if data.is_empty() {
            return Err(NanoError::InvalidParameter("Empty data".into()));
        }

        let decompressed = if data[0] == 1 {
            Self::decompress(data)?
        } else {
            data[1..].to_vec()
        };

        self.inner.decode(&decompressed)
    }

    fn name(&self) -> &'static str {
        "compressed"
    }
}

/// Frame a message with length prefix
pub fn frame_message(data: &[u8]) -> Vec<u8> {
    let len = data.len() as u32;
    let mut framed = Vec::with_capacity(4 + data.len());
    framed.extend_from_slice(&len.to_be_bytes());
    framed.extend_from_slice(data);
    framed
}

/// Extract framed message
pub fn extract_frame(data: &[u8]) -> NanoResult<(&[u8], &[u8])> {
    if data.len() < 4 {
        return Err(NanoError::InvalidParameter("Frame too short".into()));
    }

    let len = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;

    if data.len() < 4 + len {
        return Err(NanoError::InvalidParameter("Incomplete frame".into()));
    }

    Ok((&data[4..4 + len], &data[4 + len..]))
}

/// Message fragmentation for large messages
pub struct Fragmenter {
    max_fragment_size: usize,
}

impl Fragmenter {
    pub fn new(max_size: usize) -> Self {
        Self {
            max_fragment_size: max_size,
        }
    }

    /// Fragment data into chunks
    pub fn fragment(&self, data: &[u8]) -> Vec<Fragment> {
        let total = (data.len() + self.max_fragment_size - 1) / self.max_fragment_size;
        let mut fragments = Vec::new();

        for (i, chunk) in data.chunks(self.max_fragment_size).enumerate() {
            fragments.push(Fragment {
                sequence: i as u32,
                total: total as u32,
                data: chunk.to_vec(),
            });
        }

        fragments
    }

    /// Reassemble fragments
    pub fn reassemble(&self, fragments: &[Fragment]) -> NanoResult<Vec<u8>> {
        if fragments.is_empty() {
            return Ok(Vec::new());
        }

        let total = fragments[0].total as usize;
        if fragments.len() != total {
            return Err(NanoError::InvalidParameter(format!(
                "Expected {} fragments, got {}",
                total,
                fragments.len()
            )));
        }

        let mut sorted: Vec<_> = fragments.iter().collect();
        sorted.sort_by_key(|f| f.sequence);

        let mut data = Vec::new();
        for fragment in sorted {
            data.extend(&fragment.data);
        }

        Ok(data)
    }
}

/// Message fragment
#[derive(Debug, Clone)]
pub struct Fragment {
    pub sequence: u32,
    pub total: u32,
    pub data: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_message() {
        let data = b"hello";
        let framed = frame_message(data);

        let (extracted, remaining) = extract_frame(&framed).unwrap();
        assert_eq!(extracted, data);
        assert!(remaining.is_empty());
    }

    #[test]
    fn test_fragmenter() {
        let fragmenter = Fragmenter::new(4);
        let data = b"hello world!";

        let fragments = fragmenter.fragment(data);
        assert_eq!(fragments.len(), 3);

        let reassembled = fragmenter.reassemble(&fragments).unwrap();
        assert_eq!(reassembled, data);
    }
}
