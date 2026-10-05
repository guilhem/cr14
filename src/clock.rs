use std::time::{Duration, Instant};

#[cfg(not(test))]
#[derive(Default)]
pub(super) struct Clock;

#[cfg(not(test))]
impl Clock {
    pub fn new() -> Self {
        Self
    }

    pub fn now(&self) -> Instant {
        Instant::now()
    }

    pub fn sleep(&self, duration: Duration) {
        std::thread::sleep(duration);
    }
}

// The production API uses Instant. Only unit tests replace actual sleeping.
#[cfg(test)]
#[derive(Clone)]
pub(super) struct Clock {
    origin: Instant,
    elapsed: std::rc::Rc<std::cell::Cell<Duration>>,
}

#[cfg(test)]
impl Default for Clock {
    fn default() -> Self {
        Self {
            origin: Instant::now(),
            elapsed: Default::default(),
        }
    }
}

#[cfg(test)]
impl Clock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn now(&self) -> Instant {
        self.origin + self.elapsed.get()
    }

    pub fn sleep(&self, duration: Duration) {
        self.elapsed.set(self.elapsed.get() + duration);
    }
}
