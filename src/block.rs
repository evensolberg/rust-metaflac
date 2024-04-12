use crate::error::{Error, ErrorKind, Result};

use byteorder::{ReadBytesExt, WriteBytesExt, BE};

use std::collections::HashMap;
use std::convert::TryInto;
use std::io::{Read, Write};
use std::iter::repeat;

/// Types of blocks. Used primarily to map blocks to block identifiers when reading and writing.
#[allow(missing_docs, clippy::module_name_repetitions)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockType {
    StreamInfo,
    Padding,
    Application,
    SeekTable,
    VorbisComment,
    CueSheet,
    Picture,
    Unknown(u8),
}

#[allow(missing_docs)]
impl BlockType {
    /// Returns the corresponding block type byte for the block type.
    const fn to_u8(self) -> u8 {
        match self {
            Self::StreamInfo => 0,
            Self::Padding => 1,
            Self::Application => 2,
            Self::SeekTable => 3,
            Self::VorbisComment => 4,
            Self::CueSheet => 5,
            Self::Picture => 6,
            Self::Unknown(n) => n,
        }
    }

    /// Returns the corresponding block type for the block type byte.
    const fn from_u8(n: u8) -> Self {
        match n {
            0 => Self::StreamInfo,
            1 => Self::Padding,
            2 => Self::Application,
            3 => Self::SeekTable,
            4 => Self::VorbisComment,
            5 => Self::CueSheet,
            6 => Self::Picture,
            n => Self::Unknown(n),
        }
    }
}

/// The parsed content of a metadata block.
#[derive(Clone, Debug)]
pub enum Block {
    /// A value containing a parsed streaminfo block.
    StreamInfo(StreamInfo),
    /// A value containing a parsed application block.
    Application(Application),
    /// A value containing a parsed cuesheet block.
    CueSheet(CueSheet),
    /// A value containing the number of bytes of padding.
    Padding(u32),
    /// A value containing a parsed picture block.
    Picture(Picture),
    /// A value containing a parsed seektable block.
    SeekTable(SeekTable),
    /// A value containing a parsed vorbis comment block.
    VorbisComment(VorbisComment),
    /// An value containing the bytes of an unknown block.
    Unknown((u8, Vec<u8>)),
}

impl Block {
    /// Attempts to read a block from the reader. Returns a tuple containing a boolean indicating
    /// if the block was the last block, the length of the block in bytes, and the new `Block`.
    ///
    /// # Errors
    ///
    /// - Returns an error if unable to read the first byte of the block.
    /// - Returns an error if unable to read the length of the block.
    /// - Returns erors if unable to get the comments, picture(s) or cuesheet.
    ///
    /// # Panics
    ///
    /// Panics if unable to read the block data.
    ///
    #[allow(clippy::cast_possible_truncation)]
    pub fn read_from(reader: &mut dyn Read) -> Result<(bool, u32, Self)> {
        let byte = reader.read_u8()?;
        let is_last = (byte & 0x80) != 0;
        let blocktype_byte = byte & 0x7F;
        let blocktype = BlockType::from_u8(blocktype_byte);
        let length = reader.read_uint::<BE>(3)? as u32; // Returns an u64, but we know it's 3 bytes so it fits in an u32.

        let mut data = Vec::new();
        reader
            .take(u64::from(length))
            .read_to_end(&mut data)
            .expect("Unable to read block data");

        let block = match blocktype {
            BlockType::StreamInfo => Self::StreamInfo(StreamInfo::from_bytes(&data[..])),
            BlockType::Padding => Self::Padding(length),
            BlockType::Application => Self::Application(Application::from_bytes(&data[..])),
            BlockType::SeekTable => Self::SeekTable(SeekTable::from_bytes(&data[..])),
            BlockType::VorbisComment => Self::VorbisComment(VorbisComment::from_bytes(&data[..])?),
            BlockType::Picture => Self::Picture(Picture::from_bytes(&data[..])?),
            BlockType::CueSheet => Self::CueSheet(CueSheet::from_bytes(&data[..])?),
            BlockType::Unknown(_) => Self::Unknown((blocktype_byte, data)),
        };

        Ok((is_last, length + 4, block))
    }

    /// Attemps to write the block to the writer. Returns the length of the block in bytes.
    ///
    /// # Errors
    ///
    /// - Returns an error if unable to write the block data.
    #[allow(clippy::cast_possible_truncation)]
    pub fn write_to(&self, is_last: bool, writer: &mut dyn Write) -> Result<u32> {
        let (content_len, contents) = match *self {
            Self::StreamInfo(ref streaminfo) => {
                let bytes = streaminfo.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::Application(ref application) => {
                let bytes = application.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::CueSheet(ref cuesheet) => {
                let bytes = cuesheet.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::Padding(size) => (size, None),
            Self::Picture(ref picture) => {
                let bytes = picture.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::SeekTable(ref seektable) => {
                let bytes = seektable.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::VorbisComment(ref vorbis) => {
                let bytes = vorbis.to_bytes();
                (bytes.len() as u32, Some(bytes))
            }
            Self::Unknown((_, ref bytes)) => (bytes.len() as u32, Some(bytes.clone())),
        };

        let mut byte: u8 = 0;
        if is_last {
            byte |= 0x80;
        }

        byte |= self.block_type().to_u8() & 0x7F;
        writer.write_u8(byte)?;
        writer.write_all(&content_len.to_be_bytes()[1..])?;

        if let Some(bytes) = contents {
            writer.write_all(&bytes[..])?;
        } else {
            let zeroes = [0; 1024];
            let mut remaining = content_len as usize;
            loop {
                if remaining <= zeroes.len() {
                    writer.write_all(&zeroes[..remaining])?;
                    break;
                }
                writer.write_all(&zeroes[..])?;
                remaining -= zeroes.len();
            }
        }

        Ok(content_len + 4)
    }

    /// Returns the corresponding block type byte for the block.
    #[must_use]
    pub const fn block_type(&self) -> BlockType {
        match *self {
            Self::StreamInfo(_) => BlockType::StreamInfo,
            Self::Application(_) => BlockType::Application,
            Self::CueSheet(_) => BlockType::CueSheet,
            Self::Padding(_) => BlockType::Padding,
            Self::Picture(_) => BlockType::Picture,
            Self::SeekTable(_) => BlockType::SeekTable,
            Self::VorbisComment(_) => BlockType::VorbisComment,
            Self::Unknown((blocktype, _)) => BlockType::Unknown(blocktype),
        }
    }
}

// StreamInfo {{{
/// A structure representing a STREAMINFO block.
#[derive(Clone, Eq, PartialEq)]
pub struct StreamInfo {
    /// The minimum block size (in samples) used in the stream.
    pub min_block_size: u16,
    /// The maximum block size (in samples) used in the stream.
    pub max_block_size: u16,
    /// The minimum frame size (in bytes) used in the stream.
    pub min_frame_size: u32,
    /// The maximum frame size (in bytes) used in the stream.
    pub max_frame_size: u32,
    ///Sample rate in Hz.
    pub sample_rate: u32,
    /// Number of channels. FLAC supports from 1 to 8 channels.
    pub num_channels: u8,
    /// Bits per sample. FLAC supports from 4 to 32 bits per sample.
    pub bits_per_sample: u8,
    /// Total samples in stream.
    pub total_samples: u64,
    /// MD5 signature of the unencoded audio data.
    pub md5: Vec<u8>,
}

impl ::std::fmt::Debug for StreamInfo {
    fn fmt(&self, out: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        write!(out, "StreamInfo {{ min_block_size: {}, max_block_size: {}, min_frame_size: {}, max_frame_size: {}, sample_rate: {}, num_channels: {}, bits_per_sample: {}, total_samples: {}, md5: {} }}", self.min_block_size, self.max_block_size, self.min_frame_size, self.max_frame_size, self.sample_rate, self.num_channels, self.bits_per_sample, self.total_samples, hex::encode(&self.md5[..]))
    }
}

impl StreamInfo {
    /// Returns a new `StreamInfo` with zero/empty values.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            min_block_size: 0,
            max_block_size: 0,
            min_frame_size: 0,
            max_frame_size: 0,
            sample_rate: 0,
            num_channels: 0,
            bits_per_sample: 0,
            total_samples: 0,
            md5: Vec::new(),
        }
    }

    /// Parses the bytes as a `StreamInfo` block.
    #[must_use]
    #[allow(clippy::missing_panics_doc, clippy::cast_possible_truncation)]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut streaminfo = Self::new();
        let mut i = 0;

        streaminfo.min_block_size = u16::from_be_bytes(
            (&bytes[i..i + 2])
                .try_into()
                .expect("Unable to convert min_block_size to u16"),
        );
        i += 2;

        streaminfo.max_block_size = u16::from_be_bytes(
            (&bytes[i..i + 2])
                .try_into()
                .expect("Unable to convert max_block_size to u16"),
        );
        i += 2;

        streaminfo.min_frame_size = (&bytes[i..i + 3])
            .read_uint::<BE>(3)
            .expect("Unable to convert min_frame_size to u32")
            as u32;
        i += 3;

        streaminfo.max_frame_size = (&bytes[i..i + 3])
            .read_uint::<BE>(3)
            .expect("Unable to convert max_frame_size to u32")
            as u32;
        i += 3;

        // first 16 bits of sample rate
        let sample_first = u16::from_be_bytes(
            (&bytes[i..i + 2])
                .try_into()
                .expect("Unable to convert sample_first to u16"),
        );
        i += 2;

        // last 4 bits of sample rate, 3 bits of channel, first bit of bits/sample
        let sample_channel_bps = bytes[i];
        i += 1;

        streaminfo.sample_rate = u32::from(sample_first) << 4 | u32::from(sample_channel_bps) >> 4;
        streaminfo.num_channels = ((sample_channel_bps >> 1) & 0x7) + 1;

        // last 4 bits of bits/sample, 36 bits of total samples
        let bps_total = (&bytes[i..i + 5])
            .read_uint::<BE>(5)
            .expect("Unable to read bps_total");
        i += 5;

        streaminfo.bits_per_sample =
            ((sample_channel_bps & 0x1) << 4 | (bps_total >> 36) as u8) + 1;
        streaminfo.total_samples = bps_total & 0xF_FF_FF_FF_FF;

        streaminfo.md5 = bytes[i..i + 16].to_vec();

        streaminfo
    }

    /// Returns a vector representation of the streaminfo block suitable for writing to a file.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        bytes.extend(self.min_block_size.to_be_bytes().iter());
        bytes.extend(self.max_block_size.to_be_bytes().iter());
        bytes.extend(self.min_frame_size.to_be_bytes()[1..].iter());
        bytes.extend(self.max_frame_size.to_be_bytes()[1..].iter());

        // first 16 bits of sample rate
        bytes.extend(((self.sample_rate >> 4) as u16).to_be_bytes().iter());

        // last 4 bits of sample rate, 3 bits of channel, first bit of bits/sample
        let byte = ((self.sample_rate & 0xF) << 4) as u8
            | (((self.num_channels - 1) & 0x7) << 1)
            | (((self.bits_per_sample - 1) >> 4) & 0x1);
        bytes.push(byte);

        // last 4 bits of bits/sample, first 4 bits of sample count
        let byte =
            (((self.bits_per_sample - 1) & 0xF) << 4) | ((self.total_samples >> 32) & 0xF) as u8;
        bytes.push(byte);

        // last 32 bits of sample count
        bytes.extend(
            ((self.total_samples & 0xFF_FF_FF_FF) as u32)
                .to_be_bytes()
                .iter(),
        );

        bytes.extend(self.md5.iter().copied());

        bytes
    }
}

impl Default for StreamInfo {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

// Application {{{
/// A structure representing an APPLICATION block.
#[derive(Clone, Eq, PartialEq)]
pub struct Application {
    /// Registered application ID.
    pub id: Vec<u8>,
    /// Application data.
    pub data: Vec<u8>,
}

impl ::std::fmt::Debug for Application {
    fn fmt(&self, out: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        write!(
            out,
            "Application {{ id: {}, data: {:?} }}",
            hex::encode(&self.id[..]),
            self.data
        )
    }
}

impl Application {
    /// Returns a new `Application` with a zero id and no data.
    #[must_use]
    pub fn new() -> Self {
        Self {
            id: Vec::new(),
            data: Vec::new(),
        }
    }

    /// Parses the bytes as an application block.
    #[must_use]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut application = Self::new();
        let mut i = 0;

        application.id = bytes[i..i + 4].to_vec();
        i += 4;

        application.data = bytes[i..].to_vec();

        application
    }

    /// Returns a vector representation of the application block suitable for writing to a file.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        bytes.extend(self.id.iter().copied());
        bytes.extend(self.data.iter().copied());

        bytes
    }
}

impl Default for Application {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

// CueSheet {{{
/// A structure representing a cuesheet track index.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CueSheetTrackIndex {
    /// Offset in samples, relative to the track offset, of the index point.
    pub offset: u64,
    /// The index point number.
    pub point_num: u8,
}

impl CueSheetTrackIndex {
    /// Returns a new `CueSheetTrackIndex` with all zero values.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            offset: 0,
            point_num: 0,
        }
    }
}

impl Default for CueSheetTrackIndex {
    fn default() -> Self {
        Self::new()
    }
}

/// A structure representing a cuesheet track.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CueSheetTrack {
    /// Track offset in samples, relative to the beginning of the FLAC audio stream. It is the
    /// offset to the first index point of the track.
    pub offset: u64,
    /// Track number.
    pub number: u8,
    /// Track ISRC. This is a 12-digit alphanumeric code.
    pub isrc: String,
    /// The track type.
    pub is_audio: bool,
    /// The pre-emphasis flag.
    pub pre_emphasis: bool,
    /// For all tracks except the lead-out track, one or more track index points.
    pub indices: Vec<CueSheetTrackIndex>,
}

impl CueSheetTrack {
    /// Returns a new `CueSheetTrack` of type audio, without pre-emphasis, and with zero/empty
    /// values.
    #[must_use]
    pub fn new() -> Self {
        Self {
            offset: 0,
            number: 0,
            isrc: String::new(),
            is_audio: true,
            pre_emphasis: false,
            indices: Vec::new(),
        }
    }
}

impl Default for CueSheetTrack {
    fn default() -> Self {
        Self::new()
    }
}

/// A structure representing a CUESHEET block.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CueSheet {
    /// Media catalog number.
    pub catalog_num: String,
    /// The number of lead-in samples.
    pub num_leadin: u64,
    /// True if the cuesheet corresponds to a compact disc.
    pub is_cd: bool,
    /// One or more tracks.
    pub tracks: Vec<CueSheetTrack>,
}

impl CueSheet {
    /// Returns a new `CueSheet` for a CD with zero/empty values.
    #[must_use]
    pub fn new() -> Self {
        Self {
            catalog_num: String::new(),
            num_leadin: 0,
            is_cd: true,
            tracks: Vec::new(),
        }
    }

    /// Parses the bytes as a cuesheet block.
    #[allow(clippy::missing_errors_doc, clippy::missing_panics_doc)]
    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        let mut cuesheet = Self::new();
        let mut i = 0;

        cuesheet.catalog_num = String::from_utf8(bytes[i..i + 128].to_vec())?;
        i += 128;

        cuesheet.num_leadin = u64::from_be_bytes(
            (&bytes[i..i + 8])
                .try_into()
                .expect("Unable to read num_leadin"),
        );
        i += 8;

        let flags = bytes[i];
        i += 1;

        cuesheet.is_cd = (flags & 0x80) != 0;

        i += 258;

        let num_tracks = bytes[i];
        i += 1;

        for _ in 0..num_tracks {
            let mut track = CueSheetTrack::new();

            track.offset = u64::from_be_bytes(
                (&bytes[i..i + 8])
                    .try_into()
                    .expect("Unable to read track offset"),
            );
            i += 8;

            track.number = bytes[i];
            i += 1;

            track.isrc = String::from_utf8(bytes[i..i + 12].to_vec())?;
            i += 12;

            let flags = bytes[i];
            i += 1;

            track.is_audio = (flags & 0x80) == 0;
            track.pre_emphasis = (flags & 0x40) != 0;

            i += 13;

            let num_indices = bytes[i];
            i += 1;

            for _ in 0..num_indices {
                let mut index = CueSheetTrackIndex::new();

                index.offset = u64::from_be_bytes(
                    (&bytes[i..i + 8])
                        .try_into()
                        .expect("Unable to read index offset"),
                );
                i += 8;

                index.point_num = bytes[i];
                i += 1;

                i += 3;

                track.indices.push(index);
            }

            cuesheet.tracks.push(track);
        }

        Ok(cuesheet)
    }

    /// Returns a vector representation of the cuesheet block suitable for writing to a file.
    #[must_use]
    #[allow(clippy::missing_panics_doc, clippy::cast_possible_truncation)]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        assert!(self.catalog_num.len() <= 128);

        bytes.extend(self.catalog_num.clone().into_bytes());
        bytes.extend(repeat(0).take(128 - self.catalog_num.len()));
        bytes.extend(self.num_leadin.to_be_bytes().iter());

        let mut flags = 0;
        if self.is_cd {
            flags |= 0x80;
        }
        bytes.push(flags);

        bytes.extend([0; 258].iter().copied());

        bytes.push(self.tracks.len() as u8);

        for track in &self.tracks {
            assert!(track.isrc.len() <= 12);

            bytes.extend(track.offset.to_be_bytes().iter());
            bytes.push(track.number);
            bytes.extend(track.isrc.clone().into_bytes().into_iter());
            bytes.extend(
                repeat(0)
                    .take(12 - track.isrc.len())
                    .collect::<Vec<u8>>()
                    .into_iter(),
            );

            let mut flags = 0;
            if !track.is_audio {
                flags |= 0x80;
            }
            if track.pre_emphasis {
                flags |= 0x40;
            }
            bytes.push(flags);

            bytes.extend([0; 13].iter().copied());

            bytes.push(track.indices.len() as u8);

            for index in &track.indices {
                bytes.extend(index.offset.to_be_bytes().iter());
                bytes.push(index.point_num);
                bytes.extend([0; 3].iter().copied());
            }
        }

        bytes
    }
}

impl Default for CueSheet {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

// Picture {{{
/// Types of pictures that can be used in the picture block.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(missing_docs)]
pub enum PictureType {
    Other,
    Icon,
    OtherIcon,
    CoverFront,
    CoverBack,
    Leaflet,
    Media,
    LeadArtist,
    Artist,
    Conductor,
    Band,
    Composer,
    Lyricist,
    RecordingLocation,
    DuringRecording,
    DuringPerformance,
    ScreenCapture,
    BrightFish,
    Illustration,
    BandLogo,
    PublisherLogo,
}

impl PictureType {
    const fn from_u32(n: u32) -> Option<Self> {
        match n {
            0 => Some(Self::Other),
            1 => Some(Self::Icon),
            2 => Some(Self::OtherIcon),
            3 => Some(Self::CoverFront),
            4 => Some(Self::CoverBack),
            5 => Some(Self::Leaflet),
            6 => Some(Self::Media),
            7 => Some(Self::LeadArtist),
            8 => Some(Self::Artist),
            9 => Some(Self::Conductor),
            10 => Some(Self::Band),
            11 => Some(Self::Composer),
            12 => Some(Self::Lyricist),
            13 => Some(Self::RecordingLocation),
            14 => Some(Self::DuringRecording),
            15 => Some(Self::DuringPerformance),
            16 => Some(Self::ScreenCapture),
            17 => Some(Self::BrightFish),
            18 => Some(Self::Illustration),
            19 => Some(Self::BandLogo),
            20 => Some(Self::PublisherLogo),
            _ => None,
        }
    }
}

/// A structure representing a PICTURE block.
#[derive(Clone, Eq, PartialEq)]
pub struct Picture {
    /// The picture type.
    pub picture_type: PictureType,
    /// The MIME type.
    pub mime_type: String,
    /// The description of the picture.
    pub description: String,
    /// The width of the picture in pixels.
    pub width: u32,
    /// The height of the picture in pixels.
    pub height: u32,
    /// The color depth of the picture in bits-per-pixel.
    pub depth: u32,
    /// For indexed-color pictures (e.g. GIF), the number of colors used, or 0 for non-indexed
    /// pictures.
    pub num_colors: u32,
    /// The binary picture data.
    pub data: Vec<u8>,
}

impl ::std::fmt::Debug for Picture {
    fn fmt(&self, out: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        write!(out, "Picture {{ picture_type: {:?}, mime_type: {}, description: {}, width: {}, height: {}, depth: {}, num_colors: {}, data: Vec<u8> ({}) }}", self.picture_type, self.mime_type, self.description, self.width, self.height, self.depth, self.num_colors, self.data.len())
    }
}

impl Picture {
    /// Returns a new `Picture` with zero/empty values.
    #[must_use]
    pub fn new() -> Self {
        Self {
            picture_type: PictureType::Other,
            mime_type: String::new(),
            description: String::new(),
            width: 0,
            height: 0,
            depth: 0,
            num_colors: 0,
            data: Vec::new(),
        }
    }

    /// Attempts to parse the bytes as a `Picture` block. Returns a `Picture` on success.
    #[allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]
    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        let mut picture = Self::new();
        let mut i = 0;

        let picture_type_u32 = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert picture_type to u32"),
        );
        picture.picture_type = if let Some(picture_type) = PictureType::from_u32(picture_type_u32) {
            picture_type
        } else {
            return Err(Error::new(ErrorKind::InvalidInput, "invalid picture type"));
        };
        i += 4;

        let mime_length = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert mime_length to u32"),
        ) as usize;
        i += 4;

        picture.mime_type = String::from_utf8(bytes[i..i + mime_length].to_vec())?;
        i += mime_length;

        let description_length = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert description_length to u32"),
        ) as usize;
        i += 4;

        picture.description = String::from_utf8(bytes[i..i + description_length].to_vec())?;
        i += description_length;

        picture.width = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert width to u32"),
        );
        i += 4;

        picture.height = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert height to u32"),
        );
        i += 4;

        picture.depth = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert depth to u32"),
        );
        i += 4;

        picture.num_colors = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert num_colors to u32"),
        );
        i += 4;

        let data_length = u32::from_be_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert data_length to u32"),
        ) as usize;
        i += 4;

        picture.data = bytes[i..i + data_length].to_vec();

        Ok(picture)
    }

    /// Returns a vector representation of the picture block suitable for writing to a file.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        bytes.extend((self.picture_type as u32).to_be_bytes().iter());

        let mime_type = self.mime_type.clone().into_bytes();
        bytes.extend((mime_type.len() as u32).to_be_bytes().iter());
        bytes.extend(mime_type);

        let description = self.description.clone().into_bytes();
        bytes.extend((description.len() as u32).to_be_bytes().iter());
        bytes.extend(description);

        bytes.extend(self.width.to_be_bytes().iter());
        bytes.extend(self.height.to_be_bytes().iter());
        bytes.extend(self.depth.to_be_bytes().iter());
        bytes.extend(self.num_colors.to_be_bytes().iter());

        let data = self.data.clone();
        bytes.extend((data.len() as u32).to_be_bytes().iter());
        bytes.extend(data);

        bytes
    }
}

impl Default for Picture {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

// SeekTable {{{
// SeekPoint {{{
/// A structure representing a seektable seek point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SeekPoint {
    /// Sample number of first sample in the target frame, or 0xFFFFFFFFFFFFFFFF for a placeholder
    /// point.
    sample_number: u64,
    /// Offset (in bytes) from the first byte of the first frame header to the first byte of the
    /// target frame's header.
    offset: u64,
    /// Number of samples in the target frame.
    num_samples: u16,
}

impl SeekPoint {
    /// Returns a new `SeekPoint` with all zero values.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sample_number: 0,
            offset: 0,
            num_samples: 0,
        }
    }

    /// Parses the bytes as a seekpoint.
    #[must_use]
    #[allow(clippy::missing_panics_doc)]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut seekpoint = Self::new();
        let mut i = 0;

        seekpoint.sample_number = u64::from_be_bytes(
            (&bytes[i..i + 8])
                .try_into()
                .expect("Unable to convert sample_number to u64"),
        );
        i += 8;

        seekpoint.offset = u64::from_be_bytes(
            (&bytes[i..i + 8])
                .try_into()
                .expect("Unable to convert offset to u64"),
        );
        i += 8;

        seekpoint.num_samples = u16::from_be_bytes((&bytes[i..i + 2]).try_into().expect(""));

        seekpoint
    }

    /// Returns a vector representation of the seekpoint suitable for writing to a file.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        bytes.extend(self.sample_number.to_be_bytes().iter());
        bytes.extend(self.offset.to_be_bytes().iter());
        bytes.extend(self.num_samples.to_be_bytes().iter());

        bytes
    }
}

impl Default for SeekPoint {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

/// A structure representing a SEEKTABLE block.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SeekTable {
    /// One or more seek points.
    pub seekpoints: Vec<SeekPoint>,
}

impl SeekTable {
    /// Returns a new `SeekTable` with no seekpoints.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            seekpoints: Vec::new(),
        }
    }

    /// Parses the bytes as a seektable.
    #[must_use]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut seektable = Self::new();
        let num_points = bytes.len() / 18;

        let mut i = 0;
        for _ in 0..num_points {
            let seekpoint = SeekPoint::from_bytes(&bytes[i..i + 18]);
            i += 18;
            seektable.seekpoints.push(seekpoint);
        }

        seektable
    }

    /// Returns a vector representation of the seektable suitable for writing to a file.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        for seekpoint in &self.seekpoints {
            bytes.extend(seekpoint.to_bytes().into_iter());
        }

        bytes
    }
}

impl Default for SeekTable {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

// VorbisComment {{{
/// A structure representing a `VORBIS_COMMENT` block.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VorbisComment {
    /// The vendor string.
    pub vendor_string: String,
    /// A map of keys to a list of their values.
    pub comments: HashMap<String, Vec<String>>,
}

impl VorbisComment {
    /// Returns a new `VorbisComment` with an empty vendor string and no comments.
    #[must_use]
    pub fn new() -> Self {
        Self {
            vendor_string: String::new(),
            comments: HashMap::new(),
        }
    }

    /// Attempts to parse the bytes as a vorbis comment block. Returns a `VorbisComment` on
    /// success.
    #[allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]
    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        let mut vorbis = Self::new();
        let mut i = 0;

        let vendor_length = u32::from_le_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert vendor_length to u32"),
        ) as usize;
        i += 4;

        vorbis.vendor_string = String::from_utf8(bytes[i..i + vendor_length].to_vec())?;
        i += vendor_length;

        let num_comments = u32::from_le_bytes(
            (&bytes[i..i + 4])
                .try_into()
                .expect("Unable to convert num_comments to u32"),
        ) as usize;
        i += 4;

        for _ in 0..num_comments {
            let comment_length = u32::from_le_bytes(
                (&bytes[i..i + 4])
                    .try_into()
                    .expect("Unable to convert comment_length to u32"),
            ) as usize;
            i += 4;

            let comments = String::from_utf8(bytes[i..i + comment_length].to_vec())?;
            i += comment_length;

            let comments_split: Vec<&str> = comments.splitn(2, '=').collect();
            let key = comments_split[0].to_ascii_uppercase();
            let value = comments_split[1].to_owned();

            vorbis
                .comments
                .entry(key)
                .or_insert_with(|| Vec::with_capacity(1))
                .push(value);
        }

        Ok(vorbis)
    }

    /// Returns a vector representation of the vorbis comment suitable for writing to a file.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        let vendor_string = self.vendor_string.clone().into_bytes();

        bytes.extend((vendor_string.len() as u32).to_le_bytes().iter());
        bytes.extend(vendor_string);

        bytes.extend(
            (self
                .comments
                .values()
                .fold(0u32, |acc, l| acc + l.len() as u32))
            .to_le_bytes()
            .iter(),
        );

        for (key, list) in &self.comments {
            for value in list {
                let comment_string = format!("{key}={value}");
                let comment = comment_string.into_bytes();
                bytes.extend((comment.len() as u32).to_le_bytes().iter());
                bytes.extend(comment.into_iter());
            }
        }

        bytes
    }

    /// Returns a reference to the vector of comments for the specified key.
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&Vec<String>> {
        self.comments.get(key)
    }

    /// Sets the comments for the specified key. Any previous values under the key will be removed.
    pub fn set<K: Into<String>, V: Into<String>>(&mut self, key: K, values: Vec<V>) {
        let key_owned = key.into();
        self.remove(&key_owned[..]);
        self.comments.insert(
            key_owned,
            values.into_iter().map(std::convert::Into::into).collect(),
        );
    }

    /// Removes the comments for the specified key.
    pub fn remove(&mut self, key: &str) {
        self.comments.remove(key);
    }

    /// Removes any matching key/value pairs.
    pub fn remove_pair(&mut self, key: &str, value: &str) {
        if let Some(list) = self.comments.get_mut(key) {
            list.retain(|s| &s[..] != value);
        }

        let mut num_values = 0;
        if let Some(values) = self.get(key) {
            num_values = values.len();
        }
        if num_values == 0 {
            self.remove(key);
        }
    }

    // Getters/Setters {{{
    /// Returns a reference to the vector of values with the ARTIST key.
    #[must_use]
    pub fn artist(&self) -> Option<&Vec<String>> {
        self.get("ARTIST")
    }

    /// Sets the values for the ARTIST key. This will result in any ARTISTSORT comment being
    /// removed.
    pub fn set_artist<T: Into<String>>(&mut self, artists: Vec<T>) {
        self.remove("ARTISTSORT");
        self.set("ARTIST", artists);
    }

    /// Removes all values with the ARTIST key. This will result in any ARTISTSORT comments being
    /// removed as well.
    pub fn remove_artist(&mut self) {
        self.remove("ARTISTSORT");
        self.remove("ARTIST");
    }

    /// Returns a reference to the vector of values with the ALBUM key.
    #[must_use]
    pub fn album(&self) -> Option<&Vec<String>> {
        self.get("ALBUM")
    }

    /// Sets the values for the ALBUM key. This will result in any ALBUMSORT comments being
    /// removed.
    pub fn set_album<T: Into<String>>(&mut self, albums: Vec<T>) {
        self.remove("ALBUMSORT");
        self.set("ALBUM", albums);
    }

    /// Removes all values with the ALBUM key. This will result in any ALBUMSORT comments being
    /// removed as well.
    pub fn remove_album(&mut self) {
        self.remove("ALBUMSORT");
        self.remove("ALBUM");
    }

    /// Returns a reference to the vector of values with the GENRE key.
    #[must_use]
    pub fn genre(&self) -> Option<&Vec<String>> {
        self.get("GENRE")
    }

    /// Sets the values for the GENRE key.
    pub fn set_genre<T: Into<String>>(&mut self, genres: Vec<T>) {
        self.set("GENRE", genres);
    }

    /// Removes all values with the GENRE key.
    pub fn remove_genre(&mut self) {
        self.remove("GENRE");
    }

    /// Returns reference to the vector of values with the TITLE key.
    #[must_use]
    pub fn title(&self) -> Option<&Vec<String>> {
        self.get("TITLE")
    }

    /// Sets the values for the TITLE key. This will result in any TITLESORT comments being
    /// removed.
    pub fn set_title<T: Into<String>>(&mut self, title: Vec<T>) {
        self.remove("TITLESORT");
        self.set("TITLE", title);
    }

    /// Removes all values with the TITLE key. This will result in any TITLESORT comments being
    /// removed as well.
    pub fn remove_title(&mut self) {
        self.remove("TITLESORT");
        self.remove("TITLE");
    }

    /// Attempts to convert the first TRACKNUMBER comment to a `u32`.
    #[must_use]
    pub fn track(&self) -> Option<u32> {
        self.get("TRACKNUMBER").and_then(|s| {
            if s.is_empty() {
                None
            } else {
                s[0].parse::<u32>().ok()
            }
        })
    }

    /// Sets the TRACKNUMBER comment.
    pub fn set_track(&mut self, track: u32) {
        self.set("TRACKNUMBER", vec![format!("{track}")]);
    }

    /// Removes all values with the TRACKNUMBER key.
    pub fn remove_track(&mut self) {
        self.remove("TRACKNUMBER");
    }

    /// Attempts to convert the first TOTALTRACKS comment to a `u32`.
    #[must_use]
    pub fn total_tracks(&self) -> Option<u32> {
        self.get("TOTALTRACKS").and_then(|s| {
            if s.is_empty() {
                None
            } else {
                s[0].parse::<u32>().ok()
            }
        })
    }

    /// Sets the TOTALTRACKS comment.
    pub fn set_total_tracks(&mut self, total_tracks: u32) {
        self.set("TOTALTRACKS", vec![format!("{total_tracks}")]);
    }

    /// Removes all values with the TOTALTRACKS key.
    pub fn remove_total_tracks(&mut self) {
        self.remove("TOTALTRACKS");
    }

    /// Returns a reference to the vector of values with the ALBUMARTIST key.
    #[must_use]
    pub fn album_artist(&self) -> Option<&Vec<String>> {
        self.get("ALBUMARTIST")
    }

    /// Sets the values for the ALBUMARTIST key. This will result in any ALBUMARTISTSORT comments
    /// being removed.
    pub fn set_album_artist<T: Into<String>>(&mut self, album_artists: Vec<T>) {
        self.remove("ALBUMARTISTSORT");
        self.set("ALBUMARTIST", album_artists);
    }

    /// Removes all values with the ALBUMARTIST key. This will result in any ALBUMARTISTSORT
    /// comments being removed as well.
    pub fn remove_album_artist(&mut self) {
        self.remove("ALBUMARTISTSORT");
        self.remove("ALBUMARTIST");
    }

    /// Returns a reference to the vector of values with the LYRICS key.
    #[must_use]
    pub fn lyrics(&self) -> Option<&Vec<String>> {
        self.get("LYRICS")
    }

    /// Sets the values for the LYRICS key.
    pub fn set_lyrics<T: Into<String>>(&mut self, lyrics: Vec<T>) {
        self.set("LYRICS", lyrics);
    }

    /// Removes all values with the LYRICS key.
    pub fn remove_lyrics(&mut self) {
        self.remove("LYRICS");
    }
    // }}}
}

impl Default for VorbisComment {
    fn default() -> Self {
        Self::new()
    }
}
//}}}

/// Iterator over FLAC stream's blocks
pub struct Blocks<R> {
    ident_read: bool,
    finished: bool,
    reader: R,
}

impl<R> Blocks<R>
where
    R: Read,
{
    /// Create new iterator over FLAC stream's blocks
    pub const fn new(reader: R) -> Self {
        Self {
            ident_read: false,
            finished: false,
            reader,
        }
    }
}

impl<R> Iterator for Blocks<R>
where
    R: Read,
{
    /// block length and block pairs
    type Item = Result<(u32, Block)>;

    fn next(&mut self) -> Option<Self::Item> {
        if !self.ident_read {
            self.ident_read = true;
            if let Err(err) = read_ident(&mut self.reader) {
                self.finished = true;
                return Some(Err(err));
            }
        }

        if self.finished {
            None
        } else {
            match Block::read_from(&mut self.reader) {
                Ok((is_last, length, block)) => {
                    self.finished = is_last;
                    Some(Ok((length, block)))
                }
                Err(err) => {
                    self.finished = true;
                    Some(Err(err))
                }
            }
        }
    }
}

/// Read from a reader until a flac file identifier is found. Returns an error if no flac identifier
/// could be found.
pub(crate) fn read_ident<R: Read>(mut reader: R) -> Result<()> {
    use std::io;

    let mut ident = [0; 4];
    reader.read_exact(&mut ident)?;

    // skip id3 v2.2, v2.3 and v2.4
    if &ident[0..3] == b"ID3" && [0x02, 0x03, 0x04].contains(&ident[3]) {
        let mut header_tail = [0; 6];
        reader.read_exact(&mut header_tail)?;
        // Header layout from the id3v2 tag spec:
        // 3 Bytes: "ID3"
        // 2 Bytes: Maj/Min version
        // 1 Byte: Flags, bit 0x10 indicates a 10-Byte footer
        // 4 Bytes: size of the Tag, excluding header and footer, taking 7 bits per byte.
        let has_footer = header_tail[1] & 0x10 > 0;
        let size = (u32::from(header_tail[2]) & 0b_0111_1111) << 21
            | (u32::from(header_tail[3]) & 0b_0111_1111) << 14
            | (u32::from(header_tail[4]) & 0b_0111_1111) << 7
            | (u32::from(header_tail[5]) & 0b_0111_1111);
        // Discard `size` bytes without allocating. See https://stackoverflow.com/questions/42243355/how-to-advance-through-data-from-the-stdioread-trait-when-seek-isnt-impleme
        if has_footer {
            io::copy(
                &mut (&mut reader).take(u64::from(size) + 10),
                &mut io::sink(),
            )?;
        } else {
            io::copy(&mut (&mut reader).take(u64::from(size)), &mut io::sink())?;
        }

        //try to read fLaC again.
        reader.read_exact(&mut ident)?;
    }

    if &ident[..] == b"fLaC" {
        Ok(())
    } else {
        Err(Error::new(
            ErrorKind::InvalidInput,
            "reader does not contain flac metadata",
        ))
    }
}
