use std::collections::HashMap;
use crate::opcodes;
use crate::bus::Bus;

bitflags! {
    /// # Status Register (P) http://wiki.nesdev.com/w/index.php/Status_flags
    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///
    #[derive(Clone)]
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000;
        const OVERFLOW          = 0b01000000;
        const NEGATIV           = 0b10000000;
    }
}

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: CpuFlags,
    pub program_counter: u16,
    pub stack_pointer: u8,
    pub bus: Bus,
}

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
   Immediate,
   ZeroPage,
   ZeroPage_X,
   ZeroPage_Y,
   Absolute,
   Absolute_X,
   Absolute_Y,
   Indirect_X,
   Indirect_Y,
   NoneAddressing,
}

pub trait Mem {
    fn mem_read(&self, addr: u16) -> u8; 

    fn mem_write(&mut self, addr: u16, data: u8);
    
    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }
}

impl Mem for CPU {
    fn mem_read(&self, addr: u16) -> u8 { 
        self.bus.mem_read(addr)
    }

    fn mem_write(&mut self, addr: u16, data: u8) { 
        self.bus.mem_write(addr, data)
    }
}
 
impl CPU {
    pub fn new(bus: Bus) -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: CpuFlags::from_bits_truncate(0b100100),
            stack_pointer: STACK_RESET,
            program_counter: 0,
            bus: bus,
        }
    }

    pub fn get_absolute_address(&self, mode: &AddressingMode, addr: u16) -> u16 {
        match mode {
            AddressingMode::ZeroPage => self.mem_read(addr) as u16,

            AddressingMode::Absolute => self.mem_read_u16(addr),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(addr);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(addr);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(addr);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(addr);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }

            _ => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage  => self.mem_read(self.program_counter) as u16,
            
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
          
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
           
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    //Helper functions
    fn update_zero_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::ZERO);
        } else {
            self.status.remove(CpuFlags::ZERO);
        }
    }

    fn update_carry_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
    }

    fn update_negative_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::NEGATIV);
        } else {
            self.status.remove(CpuFlags::NEGATIV);
        }
    }

    fn update_overflow_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        self.update_zero_flag(result == 0);
        self.update_negative_flag(result & 0b1000_0000 != 0)
    }

    fn set_register_a(&mut self, value: u8) {
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn set_register_x(&mut self, value: u8) {
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn set_register_y(&mut self, value: u8) {
        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn add_to_register_a(&mut self, data: u8) {
        let sum = self.register_a as u16
            + data as u16
            + (if self.status.contains(CpuFlags::CARRY) {
                1
            } else {
                0
            }) as u16;

        let carry = sum > 0xff;

        if carry {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }

        let result = sum as u8;

        if (data ^ result) & (result ^ self.register_a) & 0x80 != 0 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW)
        }

        self.set_register_a(result);
    }

    fn sub_from_register_a(&mut self, data: u8) {
        self.add_to_register_a(((data as i8).wrapping_neg().wrapping_sub(1)) as u8);
    }

    fn and_with_register_a(&mut self, data: u8) {
        self.set_register_a(data & self.register_a);
    }

    fn xor_with_register_a(&mut self, data: u8) {
        self.set_register_a(data ^ self.register_a);
    }

    fn or_with_register_a(&mut self, data: u8) {
        self.set_register_a(data | self.register_a);
    }

    fn compare(&mut self, mode: &AddressingMode, compare_with: u8) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        if data <= compare_with {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }

        self.update_zero_and_negative_flags(compare_with.wrapping_sub(data));
     }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read((STACK as u16) + self.stack_pointer as u16)
    }

    fn stack_push(&mut self, data: u8) {
        self.mem_write((STACK as u16) + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1)
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;

        hi << 8 | lo
    }

    // End of Helpers ============

    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.add_to_register_a(value);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.add_to_register_a((value as i8).wrapping_neg().wrapping_sub(1) as u8);
    }

    fn lda(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_a(value);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_x(value);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_y(value);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn inc(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        value = value.wrapping_add(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
        value
    }
  
    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    
    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn and(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a & value);
    }

    fn ora(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a | value);
    }
    
    fn eor(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a ^ value);
    }

    fn asl_accumulator(&mut self){
        let mut data = self.register_a;
        self.update_carry_flag(data >> 7 == 1);
        data = data << 1;
        self.set_register_a(data)
    }

    fn asl(&mut self, mode: &AddressingMode) -> u8{
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);

        self.update_carry_flag(data >> 7 == 1);
        data = data << 1;
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn lsr_accumulator(&mut self){
        let mut data = self.register_a;
        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        self.set_register_a(data)
    }

    fn lsr(&mut self, mode: &AddressingMode) -> u8{
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);

        self.update_carry_flag(data & 1 == 1);

        data = data >> 1;
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn rol_accumulator(&mut self){
        let mut data = self.register_a;
        let old_c = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data >> 7 == 1);

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.set_register_a(data);
    }

    fn rol(&mut self, mode: &AddressingMode) -> u8{
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_c = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data >> 7 == 1);

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn ror_accumulator(&mut self){
        let mut data = self.register_a;
        let old_carry = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }
        self.set_register_a(data);
    }

    fn ror(&mut self, mode: &AddressingMode) -> u8{
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_carry = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn php(&mut self) {
        //http://wiki.nesdev.com/w/index.php/CPU_status_flag_behavior
        let mut flags = self.status.clone();
        flags.insert(CpuFlags::BREAK);
        flags.insert(CpuFlags::BREAK2);
        self.stack_push(flags.bits());
    }

    fn plp(&mut self) {
        self.status = CpuFlags::from_bits_truncate(self.stack_pop());
        self.status.remove(CpuFlags::BREAK);
        self.status.insert(CpuFlags::BREAK2);
    }

    fn bit(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);

        self.update_zero_flag(self.register_a & data == 0);
        self.update_negative_flag(data & 0b10000000 > 0);
        self.update_overflow_flag(data & 0b01000000 > 0);
    }

    fn dec(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        value = value.wrapping_sub(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn dex(&mut self){
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    
    fn dey(&mut self){
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn jmp_abs(&mut self) {
        let mem_address = self.mem_read_u16(self.program_counter);
        self.program_counter = mem_address;
    }
    
    fn jmp_ind(&mut self) {
        let mem_address = self.mem_read_u16(self.program_counter);
        let indirect_ref = if mem_address & 0x00FF == 0x00FF {
            let lo = self.mem_read(mem_address);
            let hi = self.mem_read(mem_address & 0xFF00);
            (hi as u16) << 8 | (lo as u16)
        } else {
            self.mem_read_u16(mem_address)
        };

        self.program_counter = indirect_ref;
    }

    //CPU loading, running, and initialization
    pub fn load(&mut self, program: Vec<u8>) {
        for i in 0..(program.len() as u16) {
            self.mem_write(0x0600 + i, program[i as usize]);
        }
        self.mem_write_u16(0xFFFC, 0x0600);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.stack_pointer = STACK_RESET;
        self.status = CpuFlags::from_bits_truncate(0b100100);

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    fn branch(&mut self, condition: bool){
        if condition {
            let jump: i8 = self.mem_read(self.program_counter) as i8;
            let jump_addr = self
                .program_counter
                .wrapping_add(1)
                .wrapping_add(jump as u16);

            self.program_counter = jump_addr;
        }
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
    }

    //Now, the client code can provide a callback that will be executed before every opcode interpretation cycle.
    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            callback(self);

            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;

            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized", code));

            match code {

                /* ADC - Add with carry */
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                /* SBC - Subtract with carry */
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }

                /* LDA - Load Accumulator*/
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                /* LDX - Load X Register */
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => {
                    self.ldx(&opcode.mode);
                }

                /* LDY - Load Y Register */
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                    self.ldy(&opcode.mode);
                }

                /* STA - Store Accumulator */
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                /* STX - Store X Register */
                0x86 | 0x96 | 0x8e => {
                    self.stx(&opcode.mode);
                }

                /* STY - Store Y Register */
                0x84 | 0x94 | 0x8c => {
                    self.sty(&opcode.mode);
                }

                /* TAX - Transfer Accumulator to X */
                0xaa => self.set_register_x(self.register_a),

                /* TAY - Transfer Accumulator to Y */
                0xa8 => self.set_register_y(self.register_a),

                /* TSX - Transfer Stack Pointer to X */
                0xba => self.set_register_x(self.stack_pointer),
                
                /* TXA - Transfer X to Accumulator */
                0x8a => self.set_register_a(self.register_x),

                /* TXS - Transfer X to Stack Pointer */
                0x9a => self.stack_pointer = self.register_x,

                /* TYA - Transfer Y to Accumulator */
                0x98 => self.set_register_a(self.register_y),
                
                /* INC - Increment Memory */
                0xe6 | 0xf6 | 0xee | 0xfe => {
                    self.inc(&opcode.mode);
                }
                /* INX - Increment X Register */
                0xe8 => self.inx(),
                /* INY - Increment Y Register */
                0xc8 => self.iny(),

                /* AND - Logical And */
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }

                /* ASL - Arithmetic Shift Left */
                0x0a => self.asl_accumulator(), 

                0x06 | 0x16 | 0x0e | 0x1e => {
                    self.asl(&opcode.mode);
                }
                
                /* LSR - Logical Shift Right */
                0x4a => self.lsr_accumulator(),

                0x46 | 0x56 | 0x4e | 0x5e => {
                    self.lsr(&opcode.mode);
                }

                /* ROL - Rotate Left */
                0x2a => self.rol_accumulator(),

                0x26 | 0x36 | 0x2e | 0x3e => {
                    self.rol(&opcode.mode);
                }

                /* ROR - Rotate Right*/
                0x6a => self.ror_accumulator(),

                0x66 | 0x76 | 0x6e | 0x7e => {
                    self.ror(&opcode.mode);
                }
                
                /* BCC - Branch if Carry Clear */
                0x90 => self.branch(!self.status.contains(CpuFlags::CARRY)),
                /* BCS - Branch if Carry Set */
                0xb0 => self.branch(self.status.contains(CpuFlags::CARRY)),
                
                /* BEQ - Branch if Equal */
                0xf0 => self.branch(self.status.contains(CpuFlags::ZERO)),
                /* BNE - Branch if Not Equal */
                0xd0 => self.branch(!self.status.contains(CpuFlags::ZERO)),

                /* BMI - Branch if Minus */
                0x30 => self.branch(self.status.contains(CpuFlags::NEGATIV)),
                /* BPL - Branch if Positive */
                0x10 => self.branch(!self.status.contains(CpuFlags::NEGATIV)),

                /* BVC - Branch if Overflow Clear */
                0x50 => self.branch(!self.status.contains(CpuFlags::OVERFLOW)),
                /* BVS - Branch if Overflow Set */
                0x70 => self.branch(self.status.contains(CpuFlags::OVERFLOW)),

                /* BIT - Bit Test */
                0x24 | 0x2c => self.bit(&opcode.mode),

                /* CMP - Compare */
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => {
                    self.compare(&opcode.mode, self.register_a);
                }
                /* CPX - Compare X Register */
                0xe0 | 0xe4 | 0xec => {
                    self.compare(&opcode.mode, self.register_x);
                }
                /* CPY - Compare Y Register */
                0xc0 | 0xc4 | 0xcc => {
                    self.compare(&opcode.mode, self.register_y);
                }

                /* DEC - Decrement Memory */
                0xc6 | 0xd6 | 0xce | 0xde => {
                    self.dec(&opcode.mode);
                }
                /* DEX - Decrement X Register */
                0xca => self.dex(),
                /* DEY - Decrement Y Register */
                0x88 => self.dey(),


                /* CLD - Clear Decimal Mode */
                0xd8 => self.status.remove(CpuFlags::DECIMAL_MODE),
                /* CLC - Clear Carry Flag */
                0x18 => self.status.remove(CpuFlags::CARRY),
                /* CLI - Clear Interrupt Disable */
                0x58 => self.status.remove(CpuFlags::INTERRUPT_DISABLE),
                /* CLV - Clear Overflow Flag */
                0xb8 => self.status.remove(CpuFlags::OVERFLOW),

                /* ORA - Logical Inclusive OR */
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }

                /* EOR - Exclusive OR */
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }

                /* PHA - Push Accumulator */
                0x48 => self.stack_push(self.register_a),

                /* PHP - Push Processor Status */
                0x08 => self.php(),

                /* PLP - Pull Processor Status */
                0x28 => self.plp(),

                /* PLA - Pull Accumulator */
                0x68 => {
                    let data = self.stack_pop();
                    self.set_register_a(data);
                }

                /* JMP - Jump */
                0x4c => self.jmp_abs(),
                0x6c => self.jmp_ind(),

                /* JSR - Jump to Subroutine */
                0x20 => {
                    self.stack_push_u16(self.program_counter + 2 - 1);
                    let target_address = self.mem_read_u16(self.program_counter);
                    self.program_counter = target_address
                } 

                /* RTS - Return from Subroutine */
                0x60 => {
                    self.program_counter = self.stack_pop_u16() + 1;
                }

                /* RTI - Return from Interrupt */
                0x40 => {
                    self.status = CpuFlags::from_bits_truncate(self.stack_pop());
                    self.status.remove(CpuFlags::BREAK);
                    self.status.insert(CpuFlags::BREAK2);

                    self.program_counter = self.stack_pop_u16();
                }

                /* SEC- Set Carry Flag */
                0x38 => self.status.insert(CpuFlags::CARRY),

                /* SED- Set Decimal Flag */
                0xf8 => self.status.insert(CpuFlags::DECIMAL_MODE),

                /* SEI- Set Interrupt Disable */
                0x78 => self.status.insert(CpuFlags::INTERRUPT_DISABLE),

                /* NOP - No Operation */
                0xea => {}

                /* BRK - Break */
                0x00 => return,

                /* =============== Begin Illegal Instructions ================== */
                
                /* DOP - Double NOP, TOP - Triple NOP */ 
                0x04 | 0x14 | 0x34 | 0x44 | 0x54 | 0x64 | 0x74 | 0x80 | 0x82 | 0x89 | 0xc2 | 0xd4 
                | 0xe2 | 0xf4 | 0x0c | 0x1c | 0x3c | 0x5c | 0x7c | 0xdc | 0xfc => {
                    /* we must do the read for all of these nop instructions value is not needed*/
                    let addr = self.get_operand_address(&opcode.mode);
                    let _data = self.mem_read(addr);
                    /* Do nothing */
                }

                /* KIL - Stop Program Counter */
                0x02 | 0x12 | 0x22 | 0x32 | 0x42 | 0x52 | 0x62 | 0x72 | 0x92 | 0xb2 | 0xd2
                | 0xf2 => { /* do nothing */ }

                0x1a | 0x3a | 0x5a | 0x7a | 0xda | 0xfa => { /* do nothing */ }

                /* DCP */
                0xc7 | 0xd7 | 0xCF | 0xdF | 0xdb | 0xd3 | 0xc3 => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let mut data = self.mem_read(addr);
                    data = data.wrapping_sub(1);
                    self.mem_write(addr, data);
                    // self._update_zero_and_negative_flags(data);
                    if data <= self.register_a {
                        self.status.insert(CpuFlags::CARRY);
                    }

                    self.update_zero_and_negative_flags(self.register_a.wrapping_sub(data));
                }

                /* RLA */
                0x27 | 0x37 | 0x2F | 0x3F | 0x3b | 0x33 | 0x23 => {
                    let data = self.rol(&opcode.mode);
                    self.and_with_register_a(data);
                }

                /* SLO */ 
                0x07 | 0x17 | 0x0F | 0x1f | 0x1b | 0x03 | 0x13 => {
                    let data = self.asl(&opcode.mode);
                    self.or_with_register_a(data);
                }

                /* SRE */
                0x47 | 0x57 | 0x4F | 0x5f | 0x5b | 0x43 | 0x53 => {
                    let data = self.lsr(&opcode.mode);
                    self.xor_with_register_a(data);
                }

                /* AXS */
                0xCB => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    let x_and_a = self.register_x & self.register_a;
                    let result = x_and_a.wrapping_sub(data);

                    if data <= x_and_a {
                        self.status.insert(CpuFlags::CARRY);
                    }
                    self.update_zero_and_negative_flags(result);

                    self.register_x = result;
                }

                /* ARR */
                0x6B => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    self.and_with_register_a(data);
                    self.ror_accumulator();
                    //todo: registers
                    let result = self.register_a;
                    let bit_5 = (result >> 5) & 1;
                    let bit_6 = (result >> 6) & 1;

                    if bit_6 == 1 {
                        self.status.insert(CpuFlags::CARRY)
                    } else {
                        self.status.remove(CpuFlags::CARRY)
                    }

                    if bit_5 ^ bit_6 == 1 {
                        self.status.insert(CpuFlags::OVERFLOW);
                    } else {
                        self.status.remove(CpuFlags::OVERFLOW);
                    }

                    self.update_zero_and_negative_flags(result);
                }

                /* SBC - same as other sbc*/
                0xeb => self.sbc(&opcode.mode),

                /* ANC - AND Byte with Accumulator */
                0x0b | 0x2b => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    self.set_register_a(data & self.register_a);
                    self.update_carry_flag(self.status.contains(CpuFlags::NEGATIV));
                }

                /* ALR - AND Byte with Accumulator then shift right one bit */
                0x4b => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    self.and_with_register_a(data);
                    self.lsr_accumulator();
                }

                /* RRA - Rotate Right Then Add Memory to Accumulator */
                0x67 | 0x77 | 0x6f | 0x7f | 0x7b | 0x63 | 0x73 => {
                    let data = self.ror(&opcode.mode);
                    self.add_to_register_a(data);
                }

                /* ISB - increase memory then subtract from a */
                0xe7 | 0xf7 | 0xef | 0xff | 0xfb | 0xe3 | 0xf3 => {
                    let data = self.inc(&opcode.mode);
                    self.sub_from_register_a(data);
                }

                /* LAX - Load Accumulator and X Register */
                0xa7 | 0xb7 | 0xaf | 0xbf | 0xa3 | 0xb3 => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    self.set_register_a(data);
                    self.register_x = self.register_a;
                }

                /* SAX - Store A AND with X */
                0x87 | 0x97 | 0x8f | 0x83 => {
                    let data = self.register_a & self.register_x;
                    let addr = self.get_operand_address(&opcode.mode);
                    self.mem_write(addr, data);
                }

                /* LXA - Load Accumulator and Set X to A */
                0xab => {
                    self.lda(&opcode.mode);
                    self.set_register_x(self.register_a);
                }

                /* XAA */
                0x8b => {
                    self.register_a = self.register_x;
                    self.update_zero_and_negative_flags(self.register_a);
                    let addr = self.get_operand_address(&opcode.mode);
                    let data = self.mem_read(addr);
                    self.and_with_register_a(data);
                }

                /* LAS */
                0xbb => {
                    let addr = self.get_operand_address(&opcode.mode);
                    let mut data = self.mem_read(addr);
                    data = data & self.stack_pointer;
                    self.register_a = data;
                    self.register_x = data;
                    self.stack_pointer = data;
                    self.update_zero_and_negative_flags(data);
                }

                /* TAS */
                0x9b => {
                    let data = self.register_a & self.register_x;
                    self.stack_pointer = data;
                    let mem_address =
                        self.mem_read_u16(self.program_counter) + self.register_y as u16;

                    let data = ((mem_address >> 8) as u8 + 1) & self.stack_pointer;
                    self.mem_write(mem_address, data)
                }

                /* AHX  Indirect Y */
                0x93 => {
                    let pos: u8 = self.mem_read(self.program_counter);
                    let mem_address = self.mem_read_u16(pos as u16) + self.register_y as u16;
                    let data = self.register_a & self.register_x & (mem_address >> 8) as u8;
                    self.mem_write(mem_address, data)
                }

                /* AHX Absolute Y*/
                0x9f => {
                    let mem_address =
                        self.mem_read_u16(self.program_counter) + self.register_y as u16;

                    let data = self.register_a & self.register_x & (mem_address >> 8) as u8;
                    self.mem_write(mem_address, data)
                }

                /* SHX */
                0x9e => {
                    let mem_address =
                        self.mem_read_u16(self.program_counter) + self.register_y as u16;

                    // todo if cross page boundry {
                    //     mem_address &= (self.x as u16) << 8;
                    // }
                    let data = self.register_x & ((mem_address >> 8) as u8 + 1);
                    self.mem_write(mem_address, data)
                }

                /* SHY */
                0x9c => {
                    let mem_address =
                        self.mem_read_u16(self.program_counter) + self.register_x as u16;
                    let data = self.register_y & ((mem_address >> 8) as u8 + 1);
                    self.mem_write(mem_address, data)
                }

                /* =============== End Illegal Instructions ================== */

                // _ => todo!(), //We don't need this anymore since every single possible opcode is implemented...
            }//End Match

            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }

        }//End loop
    }//End run
}//End CPU

