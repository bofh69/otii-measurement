use serial;
use std::collections::HashMap;
use structopt::StructOpt;

// TODO: Error handling, many if Some() .. tests without else
// TODO: Structure the code better, use submodules.
// TODO: Handle low voltage and when the main volt is dropped due
// to shorts.
// TODO: Put Voltage in a wrapper

const SERIAL_SETTINGS: serial::PortSettings = serial::PortSettings {
    baud_rate: serial::Baud115200,
    char_size: serial::Bits8,
    parity: serial::ParityNone,
    stop_bits: serial::Stop1,
    flow_control: serial::FlowNone,
};

const SAMPLES_PER_SECOND: u32 = 1000;

enum GpiPort {
    GPI1,
    GPI2,
}

enum Unit {
    Ampere,
    MilliAmpere,
    MicroAmpere,
}

/// Measure current with Otii Arc measurement tool
#[derive(StructOpt)]
struct Cli {
    // TODO: Change to some timespan
    /// Give up measurement after this many minutes
    #[structopt(long, default_value = "20")]
    timeout: u32,

    /// Measure for this many seconds
    #[structopt(name = "measurement-time", long, default_value = "1")]
    measurement_time: u32,

    /// The main voltage
    #[structopt(short, long, default_value = "3.3")]
    volt: f32,

    /// Voltage for digital output
    ///
    /// If not given, use main voltage
    ///
    #[structopt(name = "dig-volt", long)]
    dig_volt: Option<f32>,

    /// TTY for the Otii Arc
    #[structopt(parse(from_os_str))]
    #[structopt(long, default_value = "/dev/serial/by-id/usb-Qoitech_Arc-if00")]
    dev: std::path::PathBuf,

    /// Output debug trace
    #[structopt(parse(from_os_str))]
    #[structopt(name = "filename", long = "debug")]
    debug_filename: Option<std::path::PathBuf>,

    /// What unit to measure in
    ///
    /// Valid options are A, mA, uA
    #[structopt(short, long, default_value = "A")]
    unit: Unit,

    /// Calibrate Otii Arc before measurement
    #[structopt(long)]
    calibrate: bool,

    /// GPI1 needs to be true before measurement
    #[structopt(long = "wait-for-GPI1")]
    wait_for_gpi1: Option<bool>,

    /// GPI2 needs to be true before measurement
    #[structopt(long = "wait-for-GPI2")]
    wait_for_gpi2: Option<bool>,
}

pub trait Tracer {
    fn read(&mut self, data: &str);
    fn write(&mut self, data: &str);
}

struct NullTracer {}

struct FileTracer {
    file: std::fs::File,
}

impl Tracer for NullTracer {
    fn read(&mut self, _: &str) {}
    fn write(&mut self, _: &str) {}
}

impl Tracer for FileTracer {
    fn read(&mut self, data: &str) {
        use std::io::Write;
        self.file
            .write_all(format!("R: {}\n", data).as_bytes())
            .unwrap();
        self.file.flush().unwrap();
    }

    fn write(&mut self, data: &str) {
        use std::io::Write;
        self.file
            .write_all(format!("W: {}\n", data).as_bytes())
            .unwrap();
        self.file.flush().unwrap();
    }
}

impl FileTracer {
    fn new(path: &std::path::PathBuf) -> Self {
        Self {
            file: std::fs::File::create(path).unwrap(),
        }
    }
}

impl std::str::FromStr for Unit {
    type Err = String;

    fn from_str(text: &str) -> std::result::Result<Self, <Self as std::str::FromStr>::Err> {
        match text {
            "A" | "a" => Ok(Unit::Ampere),
            "mA" | "ma" | "m" => Ok(Unit::MilliAmpere),
            "uA" | "ua" | "µA" | "u" | "µ" => Ok(Unit::MicroAmpere),
            _ => Err(format!("Unknown unit (valid are A, mA, uA): {}", text)),
        }
    }
}

pub struct Otii {
    port: std::io::BufReader<serial::SystemPort>,
    tracer: Box<dyn Tracer>,
}

impl Otii {
    pub fn new(dev: &std::path::PathBuf, tracer: Box<dyn Tracer>) -> Otii {
        use serial::SerialPort;
        let mut port = std::io::BufReader::new(serial::open(dev).unwrap());
        port.get_mut().configure(&SERIAL_SETTINGS).unwrap();
        port.get_mut()
            .set_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        Otii { port, tracer }
    }

    fn read_line(&mut self) -> Option<String> {
        use std::io::BufRead;
        let mut buff = "".to_string();
        if self.port.read_line(&mut buff).is_ok() {
            self.tracer.read(&buff.trim());
            Some(buff)
        } else {
            None
        }
    }

    fn write_line(&mut self, text: &str) {
        use std::io::Write;
        self.port
            .get_mut()
            .write_all(format!("{}\n", text).as_bytes())
            .unwrap();
        self.tracer.write(text);
    }

    fn send_and_wait(&mut self, cmd: &str, result: &str) -> String {
        self.write_line(cmd);
        loop {
            if let Some(line) = self.read_line() {
                let mut iter = line.trim().split(':');
                if Some(result) == iter.next() {
                    if let Some(value) = iter.next() {
                        return String::from(value);
                    }
                }
            }
        }
    }

    pub fn get_gain(&mut self, channel: &str) -> f64 {
        self.write_line(&format!("getchgain {}", channel));
        loop {
            if let Some(line) = self.read_line() {
                let mut iter = line.trim().split(':');
                if Some("getchgain") == iter.next() && Some("ok") == iter.next() {
                    if let Some(result) = iter.next() {
                        if Some(channel) == iter.next() {
                            return result.parse::<f64>().unwrap();
                        }
                    }
                }
            }
        }
    }

    pub fn send_and_expect_ok(&mut self, cmd: &str, result: Option<&str>) {
        let result = if let Some(result) = result {
            result
        } else {
            cmd.split(' ').next().unwrap()
        };
        if self.send_and_wait(cmd, result) != "ok" {
            panic!("Wrong result for {}", cmd);
        }
    }

    fn parse_data_line<'a>(line: &'a [&str]) -> (u32, HashMap<&'a str, &'a str>) {
        let sample: u32 = line[0].parse().unwrap();
        let mut values = HashMap::new();
        for mut iter in line[1..].iter().map(|s| s.split('=')) {
            let key = iter.next().unwrap();
            let value = iter.next().unwrap();
            values.insert(key, value);
        }
        (sample, values)
    }

    fn wait_for_gpi(&mut self, port: GpiPort, target: bool) {
        use std::time::Instant;

        let mut last_sample = 0u32;
        let field = match port {
            GpiPort::GPI1 => "i1",
            GpiPort::GPI2 => "i2",
        };
        let target_value = if target { "1" } else { "0" };
        write_to_user(&format!(
            "Waiting for gp{} to become {}",
            field, target_value
        ));
        let start = Instant::now();

        loop {
            if let Some(line) = self.read_line() {
                let mut data = line.trim().split(':');
                if Some("d") == data.next() {
                    let vec = data.collect::<Vec<&str>>();
                    let (this_sample, values) = Self::parse_data_line(&vec);
                    if last_sample != 0u32 && last_sample != (this_sample - 1) {
                        write_to_user(&format!("Missed {} samples", this_sample - last_sample - 1));
                    }
                    last_sample = this_sample;
                    if values[field] == target_value {
                        write_to_user(&format!("Waited {} seconds.", start.elapsed().as_secs()));
                        break;
                    }
                }
            }
        }
    }

    fn wait_for_gpi1(&mut self, target: bool) {
        self.wait_for_gpi(GpiPort::GPI1, target)
    }

    fn wait_for_gpi2(&mut self, target: bool) {
        self.wait_for_gpi(GpiPort::GPI2, target)
    }

    // TODO Rewrite to use closure for the actual parsing to remove
    // code duplication with wait_for_gpi
    fn get_average_current(&mut self, max_samples: u32) -> f64 {
        let mut samples = 1;
        let mut last_sample = 0;
        let mut total_value = 0.0;
        let mut nr_values = 0;
        let pb = indicatif::ProgressBar::new(u64::from(max_samples));
        pb.set_style(indicatif::ProgressStyle::default_bar().progress_chars("█▉▊▋▌▍▎▏  "));
        loop {
            if let Some(line) = self.read_line() {
                let mut data = line.trim().split(':');
                if Some("d") == data.next() {
                    let vec = data.collect::<Vec<&str>>();
                    let (this_sample, values) = Self::parse_data_line(&vec);
                    if last_sample != 0u32 && last_sample != (this_sample - 1) {
                        write_to_user(&format!("Missed {} samples", this_sample - last_sample - 1));
                    }
                    last_sample = this_sample;

                    let mut mc_values = values["mc"].split(',');

                    let mut tmp_values = 0.0;
                    let mut high_value = None;
                    let mut tmp_nrs = 0;
                    for _ in 0..4 {
                        if let Some(sample_val) = mc_values.next() {
                            if let Ok(sample_value) = sample_val.parse::<f64>() {
                                tmp_values += sample_value;
                                tmp_nrs += 1;
                            }
                        }
                    }
                    if let Some(val) = mc_values.next() {
                        if let Ok(sample_value) = val.parse::<f64>() {
                            high_value = Some(sample_value);
                        }
                    }
                    if high_value.is_some() && Some("H") == mc_values.next() {
                        total_value += high_value.unwrap() * f64::from(tmp_nrs);
                    } else {
                        total_value += tmp_values;
                    }
                    nr_values += tmp_nrs;
                    samples += 1;
                    pb.inc(1);
                    pb.set_message(&format!(
                        "{}/{}s",
                        indicatif::HumanDuration(std::time::Duration::from_secs(u64::from(
                            samples / SAMPLES_PER_SECOND
                        ))),
                        indicatif::HumanDuration(std::time::Duration::from_secs(u64::from(
                            max_samples
                        )))
                    ));

                    if samples > max_samples {
                        break;
                    }
                }
            }
        }
        pb.finish();
        total_value / f64::from(nr_values)
    }

    pub fn init(&mut self, calibrate: bool, volt: f32, dig_volt: f32) {
        self.send_and_expect_ok("init", None);
        self.send_and_expect_ok("getuid", None);
        self.send_and_expect_ok("getname", None);
        self.send_and_expect_ok("version", None);
        self.send_and_expect_ok("led 250:250", None);
        if calibrate {
            write_to_user("Calibrating...");
            self.send_and_expect_ok("calibrate", None);
            write_to_user("Done");
        }
        // Measure currents up to 18mA
        self.send_and_expect_ok("range low", None);

        // Turn on current measurement
        self.send_and_expect_ok("setch mc", None);
        // We dont use voltage measurement
        // send_and_expect_ok("setch mv");

        // Turn on digital input channels
        self.send_and_expect_ok("setch i1", None);
        self.send_and_expect_ok("setch i2", None);

        // At most 500mA
        self.send_and_expect_ok("setmaxcur 500", None);

        self.send_and_expect_ok(&format!("expvolt {:.0}", dig_volt * 1000.0), None);
        self.send_and_expect_ok(&format!("mainvolt {:.0}", volt * 1000.0), None);
        self.send_and_expect_ok("commit", None);
    }

    fn start_measurement(&mut self) {
        self.send_and_expect_ok("led 1", None);
        self.send_and_expect_ok("start", None);
        self.send_and_expect_ok("main on", None);
    }

    fn stop_measurement(&mut self) {
        self.send_and_expect_ok("main off", None);
        self.send_and_expect_ok("led 0", None);
        self.send_and_expect_ok("stop", None);
    }
}

fn write_to_user(text: &str) {
    eprintln!("{}", text);
}

fn write_result(avg: f64, unit: Unit) {
    let mut avg = avg;
    let unit = match unit {
        Unit::Ampere => "A",
        Unit::MilliAmpere => {
            avg *= 1_000.0;
            "mA"
        }
        Unit::MicroAmpere => {
            avg *= 1_000_000.0;
            "µA"
        }
    };

    println!("{} {}", avg, unit);
}

fn get_tracer(debug_filename: Option<std::path::PathBuf>) -> Box<dyn Tracer> {
    if let Some(filename) = debug_filename {
        write_to_user(&format!("Writing debug trace to {:?}", filename));
        Box::new(FileTracer::new(&filename))
    } else {
        Box::new(NullTracer {})
    }
}

fn main() {
    let mut args = Cli::from_args();

    if args.dig_volt.is_none() {
        args.dig_volt = Some(args.volt)
    }

    // Handle timeout:
    let timer = timer::Timer::new();
    // If _guard is dropped, the timer is cancelled
    let _guard =
        timer.schedule_with_delay(chrono::Duration::minutes(i64::from(args.timeout)), || {
            write_to_user("Measurement took too long time, aborting...");
            std::process::exit(1);
        });

    let tracer = get_tracer(args.debug_filename);

    let mut otii = Otii::new(&args.dev, tracer);

    otii.init(args.calibrate, args.volt, args.dig_volt.unwrap());

    let mc_gain = otii.get_gain("mc");

    otii.start_measurement();

    if let Some(value) = args.wait_for_gpi1 {
        otii.wait_for_gpi1(value);
    }
    if let Some(value) = args.wait_for_gpi2 {
        otii.wait_for_gpi2(value);
    }
    // Flush some measurement.
    otii.get_average_current(500);

    write_to_user(&format!(
        "Starting measurement for {} seconds",
        args.measurement_time
    ));
    let start_measurement_time = std::time::Instant::now();
    let avg = otii.get_average_current(args.measurement_time * SAMPLES_PER_SECOND) / mc_gain as f64;
    write_to_user(&format!(
        "Done. Measurement took {} seconds",
        start_measurement_time.elapsed().as_secs()
    ));
    otii.stop_measurement();

    write_result(avg, args.unit);
}
