#![allow(unused,warnings)]
// look we changed the thing!

use nalgebra as nal;
use std::path;

use ggez::{event, timer, conf, graphics};
use ggez::nalgebra as na;
use ggez::{Context, GameResult};
use ggez::graphics::{DrawParam, DrawMode, Rect, MeshBuilder, Color};
use ggez::input::keyboard::{self, KeyCode, KeyMods};

use ncollide2d::shape::{Cuboid, ShapeHandle};
use ncollide2d::pipeline::narrow_phase::ContactEvent;
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, DefaultBodySet, DefaultColliderSet, Ground,
    RigidBodyDesc, ColliderDesc, RigidBody, Collider, DefaultBodyHandle,
    DefaultColliderHandle,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

type Point2 = na::Point2<f32>;
type Vector2 = na::Vector2<f32>;

const PADDLE_LIFE: f32 = 1.0;
const BLOCK_LIFE: f32 = 3.0;
const BALL_LIFE: f32 = 1.0;

const WINDOW_DIM_X: f32 = 640.0;
const WINDOW_DIM_Y: f32 = 480.0;

const BLOCK_SCALE_X: f32 = 0.5;
const BLOCK_SCALE_Y: f32 = 0.30;

const PADDLE_THRUST: f32 = 8000.0;
const BALL_THRUST: f32 = 500.0;
const PADDLE_MAX_VEL: f32 = 400.0;
const BALL_MAX_VEL: f32 = 250.0;

const BALL_RADIUS: f32 = 8.0;

enum Axis { X, Y, None, }

struct PhysicsWorld {
    mechanical: DefaultMechanicalWorld<f32>,
    geometrical: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
}
#[derive(Debug)]
enum ActorType {
    Paddle,
    Block,
    Ball,
}
#[derive(Debug)]
struct Actor {
    tag: ActorType,
    pos: Point2,
    size: Point2,
    color: Color,
    facing: f32,
    velocity: Vector2,
    bbox_size: Vector2,
    life: f32,
    stuck: bool,
}
struct Assets {
    paddle_img: graphics::Image,
    block_img: graphics::Image,
    ball_img: graphics::Image,
}
impl Assets {
    fn new(ctx: &mut Context) -> GameResult<Assets> {
        let paddle_img = graphics::Image::new(ctx, "/paddle.png")?;
        let block_img = graphics::Image::new(ctx, "/block.png")?;
        let ball_img = graphics::Image::new(ctx, "/ball.png")?;

        Ok(Assets {
            paddle_img,
            block_img,
            ball_img,
        })
    }
    fn actor_image(&mut self, actor: &Actor) -> &mut graphics::Image {
        match actor.tag {
            ActorType::Paddle => &mut self.paddle_img,
            ActorType::Block => &mut self.block_img,
            ActorType::Ball => &mut self.ball_img,
        }
    }
    fn actor_image_clone(&mut self, actor: &Actor) -> graphics::Image {
        match actor.tag {
            ActorType::Paddle => self.paddle_img.clone(),
            ActorType::Block => self.block_img.clone(),
            ActorType::Ball => self.ball_img.clone(),
        }
    }
    fn actor_image_size(&mut self, actor: &Actor) -> Point2 {
        match actor.tag {
            ActorType::Paddle => Point2::new(
                self.paddle_img.width() as f32 * BLOCK_SCALE_X,
                self.paddle_img.height() as f32 * BLOCK_SCALE_Y,
            ),
            ActorType::Block => Point2::new(
                self.block_img.width() as f32 * BLOCK_SCALE_X,
                self.block_img.height() as f32 * BLOCK_SCALE_Y,
            ),
            ActorType::Ball => Point2::new(
                self.ball_img.width() as f32 * BLOCK_SCALE_X,
                self.ball_img.height() as f32 * BLOCK_SCALE_Y,
            ),
        }
    }
    fn actor_image_color(&mut self, actor: &Actor) -> graphics::Color {
        match actor.tag {
            ActorType::Paddle => graphics::Color::new(1.0, 1.0, 1.0, 1.0),
            ActorType::Block => graphics::Color::new(1.0, 0.8, 0.0, 1.0),
            ActorType::Ball => graphics::Color::new(1.0, 1.0, 1.0, 1.0),
        }
    }
}
#[derive(Debug)]
struct InputState {
    xaxis: f32,
    yaxis: f32,
    stuck: bool,
}
impl Default for InputState {
    fn default() -> Self {
        InputState {
            xaxis: 0.0,
            yaxis: 0.0,
            stuck: true,
        }
    }
}
fn player_handle_input(actor: &mut Actor, input: &InputState, dt: f32) {
    if input.xaxis > 0.0 {
        actor.facing = std::f32::consts::PI / 2.0;
        player_thrust(actor, dt);
    } else if input.xaxis < 0.0 {
        actor.facing = std::f32::consts::PI * 1.5;
        player_thrust(actor, dt);
    }
    else {
        actor.velocity.x = 0.0;
    }
}
fn player_thrust(actor: &mut Actor, dt: f32) {
    let mut direction_vector = vec_from_angle(actor.facing);
    match actor.tag {
        ActorType::Paddle => {
            direction_vector.y = 0.0;
            direction_vector *= (PADDLE_THRUST);
        },
        ActorType::Ball => { direction_vector *= (BALL_THRUST); },
        _ => {},
    }
//  direction_vector * (PADDLE_THRUST) is the thrust vector
    actor.velocity += direction_vector * (dt);
}
const MAX_VEL: f32 = 200.0;
fn update_actor_position(actor: &mut Actor, input: &InputState, dt: f32, max_vel: f32) {
    let norm_sq = actor.velocity.norm_squared();
    if norm_sq > max_vel.powi(2) {
        actor.velocity = actor.velocity / norm_sq.sqrt() * max_vel;
    }
    let dv = actor.velocity * (dt);
    actor.pos += dv;

    match actor.tag {
        ActorType::Ball => {
            let fd = screen_bind(actor);
            reflect_facing(actor, fd);
        },
        ActorType::Paddle => {
            screen_bind(actor);
        },
        _ => {},
    }
}
fn screen_bind(actor: &mut Actor) -> Axis {
    if actor.pos.y + (actor.size.y / 2.0) >= (WINDOW_DIM_Y / 2.0) {
        actor.velocity.y = -actor.velocity.y;
        actor.pos.y = (WINDOW_DIM_Y / 2.0) - (actor.size.y / 2.0);
        Axis::Y
    }
    else if actor.pos.x - (actor.size.x / 2.0) <= -(WINDOW_DIM_X / 2.0) {
        actor.velocity.x = -actor.velocity.x;
        actor.pos.x = -(WINDOW_DIM_X / 2.0) + (actor.size.x / 2.0);
        Axis::X
    }
    else if actor.pos.x + (actor.size.x / 2.0) >= (WINDOW_DIM_X / 2.0) {
        actor.velocity.x = -actor.velocity.x;
        actor.pos.x = (WINDOW_DIM_X / 2.0) - (actor.size.x / 2.0);
        Axis::X
    }
    else { Axis::None }
}
fn reflect_facing(actor: &mut Actor, axis: Axis) {
    let pi = std::f32::consts::PI;
    match axis {
        Axis::None => {},
        Axis::X => {
            actor.facing = 2.0 * pi - actor.facing;
        },
        Axis::Y => {
            if actor.facing < pi {
                actor.facing = pi - actor.facing;
            } else {
                actor.facing =
                    (2.0 * pi - actor.facing) + pi;
            }
        },
    }
}
fn vec_from_angle(angle: f32) -> Vector2 {
    let vx = angle.sin();
    let vy = angle.cos();
    Vector2::new(vx, vy)
}
fn world_to_screen_coords(screen_width: f32, screen_height: f32, point: Point2) -> Point2 {
    let x = point.x + screen_width / 2.0;
    let y = screen_height - (point.y + screen_height / 2.0);
    Point2::new(x, y)
}
fn create_paddle() -> Actor {
    Actor {
        tag: ActorType::Paddle,
        pos: Point2::origin(),
        size: Point2::new(0.0, 0.0),
        color: Color::new(1.0, 1.0, 1.0, 1.0),
        facing: 0.,
        velocity: na::zero(),
        bbox_size: Vector2::new(0.0, 0.0),
        life: PADDLE_LIFE,
        stuck: false,
    }
}
fn create_block() -> Actor {
    Actor {
        tag: ActorType::Block,
        pos: Point2::origin(),
        size: Point2::new(64.0, 38.4),
        color: Color::new(0.8, 0.4, 0.0, 1.0),
        facing: 0.,
        velocity: na::zero(),
        bbox_size: Vector2::new(0.0, 0.0),
        life: BLOCK_LIFE,
        stuck: false,
    }
}
fn create_ball() -> Actor {
    Actor {
        tag: ActorType::Ball,
        pos: Point2::origin(),
        size: Point2::new(0.0, 0.0),
        color: Color::new(1.0, 1.0, 1.0, 1.0),
        facing: std::f32::consts::PI / 6.0,
//        facing: std::f32::consts::PI / 0.6,
        velocity: na::zero(),
        bbox_size: Vector2::new(0.0, 0.0),
        life: BALL_LIFE,
        stuck: true,
    }
}
fn create_level(rows:i32, cols:i32, assets: &mut Assets) -> Vec::<Actor> {
    let top_left = [-(WINDOW_DIM_X / 2.0), (WINDOW_DIM_Y / 2.0)];
    let mut pos: Point2 = top_left.into();
    let mut level: Vec<Actor> = Vec::new();
    let mut life = 3.0;
    let new_block = |_| {
        let mut block = create_block();
        block.life = life;
        block.pos = Point2::new(
            pos.x + (block.size.x * 0.5),
            pos.y - (block.size.y * 0.5),
        );
        pos.x += block.size.x;
        if pos.x >= (WINDOW_DIM_X / 2.0) {
            pos.y -= block.size.y;
            if pos.y >= WINDOW_DIM_Y / 2.0 {
                return block;
            }
            pos.x = top_left[0];
            if life > 1.0 { life -= 1.0; }
        }
        block
    };
    (0..(rows*cols)).map(new_block).collect()
}
struct Handle {
    col: Vec<DefaultColliderHandle>,
    rb: Vec<DefaultBodyHandle>,
}
impl Handle {
    fn new() -> Handle {
        Handle {
            col: Vec::new(),
            rb: Vec::new(),
        }
    }
}
struct MainState {
    block1: Actor,
    block2: Actor,
    handle1: Handle,
    handle2: Handle,
    assets: Assets,
    input: InputState,
    screen_width: f32,
    screen_height: f32,
    p_world: PhysicsWorld,
}
impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        let mut block1 = create_block();
        let mut block2 = create_block();
        block2.pos.y += 100.0;
        let mut assets = Assets::new(ctx)?;
        let (width, height) = graphics::drawable_size(ctx);
        let mut p_world = PhysicsWorld {
            mechanical: DefaultMechanicalWorld::new(nal::Vector2::new(0.0, -9.81)),
            geometrical: DefaultGeometricalWorld::new(),
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),
        };
        let mut s = MainState {
            block1,
            block2,
            handle1: Handle::new(),
            handle2: Handle::new(),
            assets,
            input: InputState::default(),
            screen_width: width,
            screen_height: height,
            p_world,
        };
        s.load_physics_handles();
        Ok(s)
    }
    fn load_physics_handles(&mut self) {
        // create the rigid bodies and add them to the set
        let mut rb_desc1 = RigidBodyDesc::new()
            .gravity_enabled(false)
            .mass(1.2)
            .position(nal::Isometry2::new(
                nal::Vector2::new(self.block1.pos.x, self.block1.pos.y), 0.0)
            );
        let mut rb_desc2 = RigidBodyDesc::new()
            .gravity_enabled(false)
            .mass(1.2)
            .position(nal::Isometry2::new(
                nal::Vector2::new(self.block2.pos.x, self.block2.pos.y), 0.0)
            );
        let rb1 = rb_desc1.build();
        let rb2 = rb_desc2.build();
        self.handle1.rb.push(self.p_world.bodies.insert(rb1));
        self.handle2.rb.push(self.p_world.bodies.insert(rb2));

        let shape1 = ShapeHandle::new(
            Cuboid::new(nal::Vector2::<f32>::new(self.block1.size.x, self.block1.size.y))
        );
        let shape2 = ShapeHandle::new(
            Cuboid::new(nal::Vector2::<f32>::new(self.block2.size.x, self.block2.size.y))
        );
        let collider_1 = ColliderDesc::new(shape1)
            .position(nal::Isometry2::new(
                nal::Vector2::new(self.block1.pos.x, self.block1.pos.y), 0.0)
            )
            .set_ccd_enabled(true)
            .build(BodyPartHandle(self.handle1.rb[0], 0));

        let collider_2 = ColliderDesc::new(shape2)
            .position(nal::Isometry2::new(
                nal::Vector2::new(self.block1.pos.x, self.block1.pos.y), 0.0)
            )
            .set_ccd_enabled(true)
            .build(BodyPartHandle(self.handle2.rb[0], 0));
        self.handle1.col.push(self.p_world.colliders.insert(collider_1));
        self.handle2.col.push(self.p_world.colliders.insert(collider_2));
    }
    fn handle_contact_events(&mut self) {
        for contact in self.p_world.geometrical.contact_events() {
            if let &ContactEvent::Started(collider1, collider2) = contact {
                print!("\nContact!\n");
            }
        }
    }
    fn update_collider_pos(&mut self) {
        self.block1.pos.y += 0.3;
        let new_pos = nal::Isometry2::new(nal::Vector2::new(self.block1.pos.x, self.block1.pos.y), 0.0);
        let body1 = self.p_world.bodies.rigid_body_mut(self.handle1.rb[0]).unwrap();
        let collider1 = self.p_world.colliders.get_mut(self.handle1.col[0]).unwrap();

        body1.set_position(new_pos);
        collider1.set_position(new_pos);
    }
}
fn draw_actor(
    assets: &mut Assets,
    ctx: &mut Context,
    actor: &Actor,
    world_coords: (f32, f32),
) -> GameResult {
    let image = assets.actor_image(actor);
    let (screen_w, screen_h) = world_coords;
    let pos = world_to_screen_coords(screen_w, screen_h, actor.pos);

    let scale = Vector2::new(
        actor.size.x / image.width() as f32,
        actor.size.y / image.height() as f32,
    );
//    print!("\nwhere is brock? {},{}\n",pos.x, pos.y);
    let drawparams = graphics::DrawParam::new()
        .dest(pos)
        .scale(scale)
        .color(actor.color);
//        .rotation(std::f32::consts::PI / 4.0)
//        .offset(Point2::new(0.5, 0.5));
    graphics::draw(ctx, image, drawparams)
}
impl ggez::event::EventHandler for MainState {
    fn update (&mut self, ctx: &mut Context) -> GameResult<()> {
        const DESIRED_FPS: u32 = 60;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            let seconds = 1.0 / (DESIRED_FPS as f32);

            self.p_world.mechanical.step(
                &mut self.p_world.geometrical,
                &mut self.p_world.bodies,
                &mut self.p_world.colliders,
                &mut self.p_world.joint_constraints,
                &mut self.p_world.force_generators,
            );
            self.update_collider_pos();
            self.handle_contact_events();
/*
            player_handle_input(&mut self.paddle, &self.input, seconds);
            player_thrust(&mut self.ball, seconds);
            update_actor_position(&mut self.paddle, &self.input, seconds, PADDLE_MAX_VEL);
            if self.input.stuck {
                self.ball.pos.x = self.paddle.pos.x;
                self.ball.pos.y = self.paddle.pos.y + self.paddle.size.y * 0.5 + self.ball.size.y * 0.5;
            } else {
                update_actor_position(&mut self.ball, &self.input, seconds, BALL_MAX_VEL);
            }
            self.handle_collisions();
            self.clear_dead_stuff();
            if self.ball.pos.y < -(WINDOW_DIM_Y / 2.0) {
                print!("\nGAME OVER!\n");
                let _ = event::quit(ctx);
            }
*/
        }
        Ok(())
    }
    fn draw (&mut self, ctx: &mut Context) -> GameResult<()> {
        let assets = &mut self.assets;
        graphics::clear(ctx, [0.1, 0.2, 0.3, 1.0].into());

        draw_actor(assets, ctx, &self.block1, (self.screen_width, self.screen_height))?;
        draw_actor(assets, ctx, &self.block2, (self.screen_width, self.screen_height))?;

        graphics::present(ctx)?;
        Ok(())
/*
        let assets = &mut self.assets;
        let radian = std::f32::consts::PI / 4.0;
        for b in &mut self.blocks {
            if b.life > 0.0 {
                draw_actor(assets, ctx, b, (self.screen_width, self.screen_height))?;
            }
        }
        draw_actor(assets, ctx, &self.paddle, (self.screen_width, self.screen_height))?;
        draw_actor(assets, ctx, &self.ball, (self.screen_width, self.screen_height))?;
*/
    }
    fn key_down_event(
        &mut self,
        ctx: &mut Context,
        keycode: KeyCode,
        _keymod: KeyMods,
        _repeat: bool,
    ) {
        match keycode {
            KeyCode::A => { self.input.xaxis = -1.0; /*print!("\nINPUT AXIS >> {}\n", self.input.xaxis);*/},
            KeyCode::D => { self.input.xaxis = 1.0; /*print!("\nINPUT AXIS >> {}\n", self.input.xaxis);*/},
            KeyCode::Space => { if self.input.stuck { self.input.stuck = !self.input.stuck; }},
            KeyCode::Escape => event::quit(ctx),
            _ => (),
        }
    }
    fn key_up_event(&mut self, _ctx: &mut Context, keycode: KeyCode, _keymod: KeyMods) {
        match keycode {
            KeyCode::A => {
                if keyboard::is_key_pressed(_ctx, KeyCode::D) {
                    self.input.xaxis = 1.0;
                }
                if self.input.xaxis == 1.0 {}
                else { self.input.xaxis = 0.0; }
            },
            KeyCode::D => {
                if keyboard::is_key_pressed(_ctx, KeyCode::A) {
                    self.input.xaxis = -1.0;
                }
                if self.input.xaxis == -1.0 {}
                else { self.input.xaxis = 0.0; }
            }
            _ => (),
        }
    }
}
enum Direction {
    UP, RIGHT, DOWN, LEFT, CENTER,
}
struct Collision(bool, Direction, Vector2);

fn handle_collision_paddle(ball: &mut Actor, paddle: &Actor, input: &InputState) {
    let collision: Collision = aabb_collision(ball, paddle);
    let pi = std::f32::consts::PI;
    if !input.stuck && collision.0 {
        reflect_facing(ball, Axis::Y);
        let distance = ball.pos.x - paddle.pos.x;
        let mut percentage = (distance / (paddle.size.x / 2.0)).abs();
        if percentage > 0.9 { percentage = 0.9; }
        else if percentage < 0.1 { percentage = 0.1; }
        let strength: f32 = 2.0;
        print!("\ndistance: {}\n", distance);
        print!("\n%: {}\n", percentage);
        ball.velocity.y = -ball.velocity.y;

        if ball.facing > pi {
            ball.facing = (2.0 * pi) - ((pi / 2.0) * percentage);
        } else {
            ball.facing = (pi/2.0) * percentage;
        }
    }
}
fn handle_collision_block(ball: &mut Actor, block: &mut Actor) {
    let collision: Collision = aabb_collision(ball, block);
    let dir: Direction = collision.1;
    let difference: Vector2 = collision.2;
    let penetration = Vector2::new(
        BALL_RADIUS - (difference.x.abs()),
        BALL_RADIUS - (difference.y.abs()),
    );
    if collision.0 {
        block.life -= 1.0;
        match dir {
            Direction::LEFT => {
                reflect_facing(ball, Axis::X);
                ball.velocity.x = -ball.velocity.x;
                ball.pos.x += penetration.x;
            },
            Direction::RIGHT => {
                reflect_facing(ball, Axis::X);
                ball.velocity.x = -ball.velocity.x;
                ball.pos.x -= penetration.x;
            },
            Direction::UP => {
                reflect_facing(ball, Axis::Y);
                ball.velocity.y = -ball.velocity.y;
                ball.pos.y -= penetration.y;
            },
            Direction::DOWN => {
                reflect_facing(ball, Axis::Y);
                ball.velocity.y = -ball.velocity.y;
                ball.pos.y += penetration.y;
            },
            _ => {print!("\nWOAH IT'S CENTER!\n");},
        }
    }
    match block.life {
        3.0 => { block.color = [0.8, 0.0, 0.8, 1.0].into(); },
        2.0 => { block.color = [1.0, 0.6, 0.0, 1.0].into(); },
        1.0 => { block.color = [0.0, 0.6, 0.8, 1.0].into(); },
        _ => {},
    }
}
fn v_direction(tgt:Vector2) -> Direction {
    let compass: [Vector2; 4] = [
        Vector2::new(0.0, 1.0),  // UP
        Vector2::new(1.0, 0.0),  // RIGHT
        Vector2::new(0.0, -1.0), // DOWN
        Vector2::new(-1.0, 0.0), // LEFT
    ];
    let mut dir = 0;
    let mut max:f32 = 0.0;
    let mut best_match = Direction::CENTER;
    for i in &compass {
        let dot_product = na::dot(&tgt.normalize(), i);
        if dot_product > max {
            max = dot_product;
            match dir {
                0 => { best_match = Direction::UP; },
                1 => { best_match = Direction::RIGHT; },
                2 => { best_match = Direction::DOWN; },
                3 => { best_match = Direction::LEFT; },
                _ => {},
            }
        }
        dir += 1;
    }
    best_match
}

fn aabb_collision(ball: &mut Actor, block: &Actor) -> Collision{
    let center = Vector2::new(ball.pos.x, ball.pos.y);
    let aabb_center = Vector2::new(block.pos.x, block.pos.y);
    let half_extents = Vector2::new(block.size.x / 2.0, block.size.y / 2.0);
    let mut difference = center - aabb_center;
    let clamped = vec2_max(-half_extents, vec2_min(half_extents, difference));
    let closest = aabb_center + clamped;
    difference = closest - center;
//    vec2_length(difference) < BALL_RADIUS
    if vec2_length(difference) <= BALL_RADIUS {
        Collision(true, v_direction(difference), difference)
    } else {
        Collision(false, Direction::UP, Vector2::new(0.0, 0.0))
    }
}
fn vec2_min(a:Vector2, b:Vector2) -> Vector2 {
    Vector2::new(a.x.min(b.x), a.y.min(b.y))
}
fn vec2_max(a:Vector2, b:Vector2) -> Vector2 {
    Vector2::new(a.x.max(b.x), a.y.max(b.y))
}
fn vec2_length(v:Vector2) -> f32 {
    ((v.x * v.x) + (v.y * v.y)).sqrt()
}
fn main() -> GameResult {
    let resource_dir = path::PathBuf::from("./resources");
    let cb = ggez::ContextBuilder::new("Breakout", "Joe")
        .window_setup(conf::WindowSetup::default().title("Breakout"))
        .window_mode(conf::WindowMode::default().dimensions(640., 480.))
        .add_resource_path(resource_dir);
    let (ctx, event_loop) = &mut cb.build()?;
    let game = &mut MainState::new(ctx)?;

    event::run(ctx, event_loop, game)

}
