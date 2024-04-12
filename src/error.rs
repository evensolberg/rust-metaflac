use std::error;
use std::fmt;
use std::io;
use std::string;

/// Type alias for the result of tag operations.
pub type Result<T> = ::std::result::Result<T, Error>;

/// Kinds of errors that may occur while performing metadata operations.
#[derive(Debug)]
#[allow(clippy::module_name_repetitions)]
pub enum ErrorKind {
    /// An error kind indicating that an IO error has occurred. Contains the original io::Error.
    Io(io::Error),
    /// An error kind indicating that a string decoding error has occurred. Contains the invalid
    /// bytes.
    StringDecoding(string::FromUtf8Error),
    /// An error kind indicating that some input was invalid.
    InvalidInput,
}

/// A structure able to represent any error that may occur while performing metadata operations.
pub struct Error {
    /// The kind of error.
    pub kind: ErrorKind,
    /// A human readable string describing the error.
    pub description: &'static str,
}

impl Error {
    /// Creates a new `Error` using the error kind and description.
    #[must_use]
    pub const fn new(kind: ErrorKind, description: &'static str) -> Self {
        Self { kind, description }
    }
}

impl error::Error for Error {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        match self.kind {
            ErrorKind::Io(ref err) => Some(err),
            ErrorKind::StringDecoding(ref err) => Some(err),
            ErrorKind::InvalidInput => None,
        }
    }

    #[allow(deprecated)]
    fn description(&self) -> &str {
        if self.source().is_some() {
            self.source()
                .expect("Unable to get lower-level source of error")
                .description()
        } else {
            match self.kind {
                ErrorKind::Io(ref err) => error::Error::description(err),
                ErrorKind::StringDecoding(ref err) => err.description(),
                ErrorKind::InvalidInput => self.description,
            }
        }
    }

    fn cause(&self) -> Option<&dyn error::Error> {
        error::Error::source(self)
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Self {
            kind: ErrorKind::Io(err),
            description: "",
        }
    }
}

impl From<string::FromUtf8Error> for Error {
    fn from(err: string::FromUtf8Error) -> Self {
        Self {
            kind: ErrorKind::StringDecoding(err),
            description: "",
        }
    }
}

impl fmt::Debug for Error {
    fn fmt(&self, out: &mut fmt::Formatter) -> fmt::Result {
        if !self.description.is_empty() {
            write!(out, "{:?}: {}", self.kind, self.description)
        } else if let Some(source) = error::Error::source(self) {
            write!(out, "{source}")
        } else {
            write!(out, "{:?}", self.kind)
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, out: &mut fmt::Formatter) -> fmt::Result {
        if !self.description.is_empty() {
            write!(out, "{:?}: {}", self.kind, self.description)
        } else if let Some(source) = error::Error::source(self) {
            write!(out, "{source}")
        } else {
            write!(out, "{:?}", self.kind)
        }
    }
}
