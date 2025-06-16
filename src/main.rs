mod globals;
mod resources;
mod ui;

use crate::globals::*;
use crate::ui::UiPlugin;
use bevy::prelude::*;
use rand::prelude::*;
use crate::resources::grid::GridSettings;

#[derive(Resource, Default)]
struct MouseWorldPosition {
    position: Option<Vec2>,
}

#[derive(Component, Default)]
struct Velocity(Vec2);

#[derive(Component)]
#[require(Transform, Velocity)]
struct Boid {
    mass: f32,
    max_speed: f32,
    max_force: f32,
}

impl Default for Boid {
    fn default() -> Self {
        Self {
            mass: BOID_MASS,
            max_speed: BOID_MAX_SPEED,
            max_force: BOID_MAX_FORCE,
        }
    }
}

#[derive(Resource)]
struct BoidSettings {
    separation_radius: f32,
    alignment_radius: f32,
    cohesion_radius: f32,
    separation_weight: f32,
    alignment_weight: f32,
    cohesion_weight: f32,
    view_angle: f32,
    mouse_attraction_weight: f32,
    mouse_arrival_radius: f32,
}

impl Default for BoidSettings {
    fn default() -> Self {
        Self {
            separation_radius: 20.0,   
            alignment_radius: 80.0,     
            cohesion_radius: 120.0,    

            separation_weight: 2.5,      
            alignment_weight: 1.8,      
            cohesion_weight: 1.0,       

            view_angle: 4.7,            
            mouse_attraction_weight: 0.1,
            mouse_arrival_radius: 30.0,
        }
    }
}

// Fonction utilitaire pour vérifier si un boid est dans le champ de vision
fn is_in_view(boid_pos: Vec2, boid_dir: Vec2, other_pos: Vec2, view_angle: f32) -> bool {
    let to_other = (other_pos - boid_pos).normalize_or_zero();
    let angle = boid_dir.dot(to_other).acos();
    angle <= view_angle / 2.0
}

// Structure pour stocker les voisins d'un boid
struct Neighbors {
    separation: Vec<(Vec2, Vec2)>, // (position, vélocité)
    alignment: Vec<(Vec2, Vec2)>,
    cohesion: Vec<(Vec2, Vec2)>,
}

fn calculate_separation(boid_pos: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // fold() est comme reduce() : accumule les valeurs
    // Ici, on additionne tous les vecteurs de répulsion
    let steer = neighbors
        .iter()
        .fold(Vec2::ZERO, |acc, &(neighbor_pos, _)| {
            let diff = boid_pos - neighbor_pos;
            let distance = diff.length();
            // Plus le voisin est proche, plus la répulsion est forte
            if distance > 0.0 {
                acc + diff.normalize() / distance
            } else {
                acc
            }
        });

    steer.normalize_or_zero()
}

fn calculate_alignment(boid_velocity: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // Calcul de la vélocité moyenne des voisins
    let avg_velocity = neighbors
        .iter()
        .map(|&(_, vel)| vel) // map() transforme chaque élément
        .sum::<Vec2>()
        / neighbors.len() as f32;

    (avg_velocity - boid_velocity).normalize_or_zero()
}

fn calculate_cohesion(boid_pos: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // Centre de masse du groupe
    let center = neighbors.iter().map(|&(pos, _)| pos).sum::<Vec2>() / neighbors.len() as f32;

    (center - boid_pos).normalize_or_zero()
}

fn boid_movement_system(
    mut boids: Query<(Entity, &mut Transform, &mut Velocity, &Boid)>,
    settings: Res<BoidSettings>,
    mouse_pos: Res<MouseWorldPosition>,
    time: Res<Time>,
) {
    let boid_data: Vec<_> = boids
        .iter()
        .map(|(entity, transform, velocity, _)| (entity, transform.translation.xy(), velocity.0))
        .collect();

    for (entity, mut transform, mut velocity, boid) in boids.iter_mut() {
        let pos = transform.translation.xy();
        let vel = velocity.0;
        let direction = vel.normalize_or_zero();

        // [Code existant pour les voisins...]
        let mut neighbors = Neighbors {
            separation: Vec::new(),
            alignment: Vec::new(),
            cohesion: Vec::new(),
        };

        for &(other_entity, other_pos, other_vel) in boid_data.iter() {
            if entity == other_entity {
                continue;
            }

            let distance = pos.distance(other_pos);
            if !is_in_view(pos, direction, other_pos, settings.view_angle) {
                continue;
            }

            if distance < settings.separation_radius {
                neighbors.separation.push((other_pos, other_vel));
            }
            if distance < settings.alignment_radius {
                neighbors.alignment.push((other_pos, other_vel));
            }
            if distance < settings.cohesion_radius {
                neighbors.cohesion.push((other_pos, other_vel));
            }
        }

        let separation =
            calculate_separation(pos, &neighbors.separation) * settings.separation_weight;
        let alignment = calculate_alignment(vel, &neighbors.alignment) * settings.alignment_weight;
        let cohesion = calculate_cohesion(pos, &neighbors.cohesion) * settings.cohesion_weight;

        // NOUVEAU : Comportement "goal seeking" avec arrivée
        let mouse_seeking = if let Some(mouse_world_pos) = mouse_pos.position {
            calculate_seek_with_arrival(
                pos,
                vel,
                mouse_world_pos,
                boid.max_speed,
                boid.max_force,
                settings.mouse_arrival_radius,
            ) * settings.mouse_attraction_weight
        } else {
            Vec2::ZERO
        };

        // Combiner toutes les forces
        let steering_force = separation + alignment + cohesion + mouse_seeking;

        // Limiter la force totale avant de l'appliquer
        let clamped_force = steering_force.clamp_length_max(boid.max_force);

        // Appliquer la physique
        let acceleration = clamped_force / boid.mass;
        velocity.0 += acceleration * time.delta_secs();
        velocity.0 = velocity.0.clamp_length_max(boid.max_speed);

        // Mettre à jour position et rotation
        transform.translation += velocity.0.extend(0.0) * time.delta_secs();

        if velocity.0.length() > 0.0 {
            let angle = velocity.0.y.atan2(velocity.0.x) - std::f32::consts::FRAC_PI_2;
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

fn border_repulsion_system(
    mut boids: Query<(&Transform, &mut Velocity), With<Boid>>,
    grid_settings: Res<GridSettings>,
    time: Res<Time>,
) {
    let border_distance = 50.0; // Distance à partir de laquelle la répulsion commence
    let repulsion_strength = 200.0; // Force de la répulsion

    for (transform, mut velocity) in boids.iter_mut() {
        let pos = transform.translation.xy();
        let mut repulsion_force = Vec2::ZERO;

        // Calcul de la distance aux bords
        let half_width = grid_settings.width / 2.0;
        let half_height = grid_settings.height / 2.0;

        // Répulsion du bord droit
        let dist_right = half_width - pos.x;
        if dist_right < border_distance {
            // Plus on est proche du bord, plus la force est grande
            let strength = (1.0 - dist_right / border_distance) * repulsion_strength;
            repulsion_force.x -= strength;
        }

        // Répulsion du bord gauche
        let dist_left = pos.x + half_width;
        if dist_left < border_distance {
            let strength = (1.0 - dist_left / border_distance) * repulsion_strength;
            repulsion_force.x += strength;
        }

        // Répulsion du bord haut
        let dist_top = half_height - pos.y;
        if dist_top < border_distance {
            let strength = (1.0 - dist_top / border_distance) * repulsion_strength;
            repulsion_force.y -= strength;
        }

        // Répulsion du bord bas
        let dist_bottom = pos.y + half_height;
        if dist_bottom < border_distance {
            let strength = (1.0 - dist_bottom / border_distance) * repulsion_strength;
            repulsion_force.y += strength;
        }

        // Appliquer la force de répulsion
        velocity.0 += repulsion_force * time.delta_secs();
    }
}

fn generate_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    grid_settings: Res<GridSettings>,
) {
    let mut rng = rand::rng();

    let mesh = meshes.add(Triangle2d::new(
        Vec2::Y * BOID_SIZE * 1.6,
        Vec2::new(-BOID_SIZE, -BOID_SIZE),
        Vec2::new(BOID_SIZE, -BOID_SIZE),
    ));

    let color = Color::WHITE;

    for _ in 0..NUMBER_BOIDS {
        let angle = rng.random::<f32>() * std::f32::consts::TAU;
        let initial_velocity = Vec2::new(angle.cos(), angle.sin()) * 80.0;

        commands.spawn((
            Boid::default(),
            Transform::from_xyz(
                rng.random_range(-grid_settings.width / 2.0..grid_settings.width / 2.0),
                rng.random_range(-grid_settings.height / 2.0..grid_settings.height / 2.0),
                0.0,
            ),
            Velocity(initial_velocity),
            Mesh2d(mesh.clone()),
            MeshMaterial2d(materials.add(color)),
        ));
    }
}

fn calculate_seek_with_arrival(
    current_pos: Vec2,
    current_vel: Vec2,
    target_pos: Vec2,
    max_speed: f32,
    max_force: f32,
    arrival_radius: f32,
) -> Vec2 {
    let to_target = target_pos - current_pos;
    let distance = to_target.length();

    if distance < 0.001 {
        return Vec2::ZERO;
    }

    let desired_speed = if distance < arrival_radius {
        let normalized_distance = distance / arrival_radius;
        max_speed * normalized_distance.sqrt()
    } else {
        max_speed
    };

    let desired_velocity = to_target.normalize() * desired_speed;
    let steering = desired_velocity - current_vel;

    // Augmenter la force quand on est loin pour garantir l'attraction
    let force_multiplier = if distance > arrival_radius { 2.0 } else { 1.0 };

    steering.clamp_length_max(max_force * force_multiplier)
}

#[derive(Component)]
struct MouseIndicator;

// Système pour créer l'indicateur de souris
fn setup_mouse_indicator(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn((
        MouseIndicator,
        Mesh2d(meshes.add(Circle::new(10.0))),
        MeshMaterial2d(materials.add(Color::srgba(1.0, 0.0, 0.0, 0.5))),
        Transform::from_xyz(0.0, 0.0, 1.0), 
    ));

    commands.spawn((
        MouseIndicator,
        Mesh2d(meshes.add(Circle::new(30.0))), 
        MeshMaterial2d(materials.add(Color::srgba(0.0, 1.0, 0.0, 0.1))),
        Transform::from_xyz(0.0, 0.0, 0.5),
    ));
}

fn update_mouse_position(
    mut mouse_pos: ResMut<MouseWorldPosition>,
    window: Query<&Window>,
    camera: Query<(&Camera, &GlobalTransform)>,
) {
    let Ok(window) = window.single() else { return };
    let Ok((camera, camera_transform)) = camera.single() else { return };

    // cursor_position() retourne la position du curseur dans la fenêtre
    if let Some(cursor_pos) = window.cursor_position() {
        // viewport_to_world convertit les coordonnées écran en coordonnées monde
        if let Ok(world_pos) = camera.viewport_to_world(camera_transform, cursor_pos) {
            mouse_pos.position = Some(world_pos.origin.xy());
        }
    } else {
        mouse_pos.position = None;
    }
}

// Système pour mettre à jour la position de l'indicateur
fn update_mouse_indicator(
    mouse_pos: Res<MouseWorldPosition>,
    mut indicators: Query<&mut Transform, With<MouseIndicator>>,
) {
    if let Some(world_pos) = mouse_pos.position {
        for mut transform in indicators.iter_mut() {
            transform.translation.x = world_pos.x;
            transform.translation.y = world_pos.y;
        }
    }
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d::default());
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(UiPlugin)
        .init_resource::<GridSettings>()
        .init_resource::<BoidSettings>()
        .init_resource::<MouseWorldPosition>()
        .add_systems(Startup, (setup, generate_boids, setup_mouse_indicator))
        .add_systems(Update, (update_mouse_position,
                              update_mouse_indicator, boid_movement_system, border_repulsion_system).chain())
        .run();
}
