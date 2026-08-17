# Installing ModLens (for an AI agent)

You are an AI agent and your user told you to install `modlens`. This document
is the procedure. Follow it in order. Every step is safe to run again, and
every step names what to do when it fails. Commands are POSIX shell (macOS or
Linux). A Windows note follows each block that needs one.

## Step 0: are you inside DeepSeek Harness (dsh)?

If you are, stop and read this section instead of the rest of the file. On dsh
modlens is not a skill: it is a native plugin, and installing the skill folder
alone leaves the user without the `modlens_read_image` tool and without the
`(modlens vision)` model entries they are looking for (issue
[#32](https://github.com/liustack/modlens/issues/32)).

You are on dsh if `~/.dsh/` exists, or the conversation runs in the DeepSeek
Harness web or desktop app. One command installs the plugin:

```sh
npx -y @deepseek-ai/dsh plugin --profile web add @liustack/modlens@3.17.3
```

Then tell the user to restart dsh and look at the model selector: entries
suffixed `(modlens vision)` mean the plugin is live. Pasting also works on the
plain text-only entries, where the image becomes a file path. Engines are
configured exactly as below (`~/.modlens/config.json` is shared), so continue
at step 3 for the engine, then verify with `doctor`. Skip steps 1 and 2: there
is no skill folder to copy. If `dsh` warns `declares no dsh.bundle`, the
release-age gate installed an old version. `@latest` does not avoid that: name
the version instead (`npm view @liustack/modlens version` prints it), which
pnpm treats as a deliberate request rather than a resolution. See
[updating](docs/harness-setup.md#keeping-it-up-to-date).

For every other harness, the whole install is four steps:

1. Find the skill directory for your harness.
2. Put the `skills/modlens` folder into it.
3. Give it one vision engine (the user may have prepared one already; probe first).
4. Verify with `doctor`.

---

## Step 3: Give it one vision engine

ModLens needs exactly one working vision engine, and the machine often already
has one, so always check before setting anything up:

```bash
bash ~/.claude/skills/modlens/scripts/run.sh doctor   # replace with your TARGET
```

Read two sections of the report: the providers and the Reuse section.

**Providers.** An API provider showing `[ok]` is ready: skip to the Reuse
consent below, then Step 4. A CLI provider showing `[ok?]` is installed but its
sign-in cannot be verified offline: treat it as probably-ready, and let the
first real read in Step 4 confirm it (a signed-out CLI fails there with its own
sign-in guidance — relay that, it is the fix). A machine with Claude Code
signed in has `claude-cli` working with zero further setup. That route is the
slow one (a 20-45 second agent loop that also spends the user's Claude
subscription), which is fine as a starting point: tell the user it already
works, and that a free Gemini key (Path 1) brings a read
down to 5-10 seconds whenever they want the upgrade.

**Reuse consent.** The Reuse section lists local harness CLIs whose logins
could read images: the harness you are installing into counts too (a signed-in
Codex, an OpenCode vision model, credentials held by pi). For each entry marked
`not asked` that has vision, ask the user one question, naming the harness and
whose quota it spends, for example: "Your Codex CLI is signed in and its model
reads images. Allow modlens to reuse it when needed? Every reused read is
labeled with whose quota it spent." Record each answer, yes or no:

```bash
bash ~/.claude/skills/modlens/scripts/run.sh config set reuse.codex true    # or false
```

A refusal recorded is a question never asked again. Skip what the user skips.

If no provider is `[ok]` and nothing was granted, pick one path below. If the
user handed you a Gemini API key, use Path 1 with it now. If they already have
an OpenAI-compatible endpoint, use Path 3.

### Path 1: Gemini API key (recommended; works in headless sessions)

Run both commands through the launcher (replace the path with your `TARGET`):

```bash
bash ~/.claude/skills/modlens/scripts/run.sh config set gemini-api.apiKey <KEY>
bash ~/.claude/skills/modlens/scripts/run.sh config set provider gemini-api
```

If you do not have a key, ask the user for one: a free key takes about three
minutes at [Google AI Studio](https://aistudio.google.com), no card required.
The key is written to `~/.modlens/config.json` with `0600` permissions, and
re-running the commands overwrites the value in place.

### Path 2: Antigravity CLI (no key; needs the user's browser sign-in)

Antigravity (`agy`) needs no API key, but its one-time browser sign-in can only
be completed by the user, and its login token lives in the OS keyring, which is
locked in most headless sessions (cron, systemd, SSH without a desktop). On
those machines use Path 1 instead. Handle it in three idempotent steps:

1. **Probe.** Is `agy` already installed?

   ```bash
   command -v agy
   ```

   If it prints a path, skip the install. If it prints nothing, install it (you
   run this, no user action needed):

   ```bash
   curl -fsSL https://antigravity.google/cli/install.sh | bash
   ```

2. **Confirm it runs.** This spends no quota and needs no login:

   ```bash
   agy --version
   ```

   `agy: command not found` here means the installer did not add `agy` to this
   shell's PATH: open a new shell, or have the user do so, then probe again.

3. **Sign-in.** `agy` has no offline way to report whether it is already signed
   in, so decide from what you just saw. If `agy` was **already installed** before
   this run, the user most likely signed in earlier: go on to Step 4, and only
   come back here if `doctor` or a real read reports a sign-in or auth error. If
   you **just installed** `agy`, it is not signed in yet: run it once, then **ask
   the user to complete the Google sign-in in the browser it opens, and wait for
   them to confirm before you continue.** Have them exit `agy` once signed in.
   You cannot do this sign-in yourself.

   ```bash
   agy   # opens the browser for the user's one-time sign-in, then they exit
   ```

**If it fails:**
- A `config set` write error -> `~/.modlens/config.json` is not writable by this
  process. Confirm the home directory is writable.
- `agy: command not found` after the installer -> the install did not add `agy`
  to this shell's PATH. Open a new shell, or have the user do so, then re-run.
- Not sure an engine is set up? Step 4 reports exactly which providers are ready.

> **Windows:** the `curl | bash` installer is for macOS and Linux. On Windows,
> agy is usable only if the tool ships a native Windows build on PATH (see
> "Platform support" in the README). The Gemini key path is pure HTTP and works
> the same on Windows.

### Path 3: An OpenAI-compatible endpoint the user already has

```bash
bash ~/.claude/skills/modlens/scripts/run.sh config set openai.baseUrl <url>
bash ~/.claude/skills/modlens/scripts/run.sh config set openai.apiKey <key>
bash ~/.claude/skills/modlens/scripts/run.sh config set openai.model <model>
bash ~/.claude/skills/modlens/scripts/run.sh config set provider openai
```

All three fields are required, and the model must accept image input (a
text-only coding model on the same platform will not work). Recipes for common
platforms live in the skill's `references/configure.md`.

---

## Step 4: Verify

Run the diagnosis through the launcher. It spends no quota. (On a machine
where the launcher resolves to npx or bunx, the first call may download the
pinned package; that is how those runners work.)

```bash
bash ~/.claude/skills/modlens/scripts/run.sh doctor   # replace with your TARGET
```

The launcher prints its runtime selection first (whether it chose a `modlens`
on PATH, `npx`, or `bunx`), then chains modlens's own report below a
`--- modlens doctor ---` line. A healthy result for the recommended Gemini setup
looks like this (trimmed):

```
Providers
  [ok] gemini-api: apiKey: file
  ...
Selected provider
  gemini-api
  reason: provider set in the config file
```

**Success is the two lines under `Selected provider`:** the provider named there
is the one that will run, and it must appear as `[ok]` in the `Providers` list
above. The other providers showing `[!!]` is normal and needs no action.

Common lines and what they mean:

| Line | Meaning | Fix |
| :-- | :-- | :-- |
| `[!!] ... (minimum 22.19)` under `Node` | Node is too old | Upgrade Node to 22.19+ |
| `Selected provider: antigravity-cli` when you configured Gemini | The provider was never switched | Re-run `config set provider gemini-api` (Step 3) |
| `[!!] gemini-api: missing: apiKey` | The key was not saved | Re-run the Step 3 Path 1 commands |
| `[!!] antigravity-cli: agy not on PATH` | Antigravity is not installed | Use Path 1, or complete Step 3 Path 2 |
| `none detected` under `Harness` | You are not inside a recognized agent right now | Fine for a plain CLI check; recovery detects the harness at run time |

Add `--json` for a machine-readable report you can parse directly.

**If it fails:**
- The launcher printed a JSON diagnosis and exited 78 -> no runtime could run
  modlens: no compatible `modlens` on PATH, no `npx`, and no `bunx`. Read the
  `nextSteps` field in that JSON and relay it. The manual fix is to install
  Node 22.19+ (https://nodejs.org) or Bun (https://bun.sh), then re-run this
  step. Do not report modlens as broken.
- Any other message -> it is catalogued with its cause and fix in
  [`docs/troubleshooting.md`](docs/troubleshooting.md).

Once the selected provider reads `[ok]`, installation is complete. The skill
triggers on its own the next time an image shows up. To confirm the read path
end to end, run the launcher on any local image (this one call does use the
engine, so it spends one read):

```bash
bash ~/.claude/skills/modlens/scripts/run.sh -i <path-to-image>
```

---

## Done

The skill is installed and a vision engine is ready. From now on you do not type
these commands by hand: the skill triggers on its own when an image needs
reading. To change engines or add a key later, see
[`skills/modlens/references/configure.md`](skills/modlens/references/configure.md).
