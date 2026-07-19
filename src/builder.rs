use crate::config::SensorConf;


pub trait SimBuilder {
	fn imu(&mut self) -> [SensorConf<3>; 3];
	fn baro(&mut self) -> SensorConf<1>;
	fn rate(&mut self) -> u32;
}