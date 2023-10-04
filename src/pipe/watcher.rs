//! A pipe buffer state watcher that can be `await`ed.



use core::{
    future::Future,

    pin::Pin,

    task::{
        Context, Poll,
    },
};

use super::*;



pub struct Watcher<'a> {
    /// The pipe that is being watched.
    pipe: &'a DataPipe,

    /// The expected pipe buffer state.
    state: BufferState,
}

impl<'a> Watcher<'a> {
    /// Creates a new watcher for the given pipe.
    pub(crate) fn new(pipe: &'a DataPipe, state: BufferState) -> Self {
        Self { pipe, state, }
    }
}

impl<'a> Future for Watcher<'a> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Check if the pipe was closed.
        if self.pipe.closed() { return Poll::Ready( Err( Error::PipeClosed ) ) };

        // Check if the pipe was orphaned.
        if self.pipe.orphan() { return Poll::Ready( Err( Error::Orphaned ) ) };

        // Check if there was an error.
        if let Some(e) = self.pipe.error { return Poll::Ready( Err( e ) ) };

        // Check if the pipe buffer state matches the expected state.
        if self.pipe.bufstate == self.state { return Poll::Ready( Ok( () ) ) }

        // If neither happenned, pend the future.
        cx.waker().wake_by_ref();

        Poll::Pending
    }
}
