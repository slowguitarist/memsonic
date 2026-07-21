use crate::config::SensorConf;

pub trait SimBuilder<'a> where Self: Default {
	fn imu(&mut self) -> &'a [SensorConf<3>; 3];
	fn baro(&mut self) -> &'a SensorConf<1>;
	fn rate(&mut self) -> u32;
}

